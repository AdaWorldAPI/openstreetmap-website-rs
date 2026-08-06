//! Tag text → codebook ordinals, resolved once at bake time.
//!
//! A `.osm.pbf` tag is a `(key, value)` pair of arbitrary strings. A row slot
//! holds a `u24` ordinal ([`IdentityQuad`]'s carving), never text — so the bake
//! owes two codebooks, one per side, and every tag becomes a pair of ordinals
//! against them. That resolution happens **once, before the bake**, which is
//! the same discipline `row::resolve_identities` already applies to element
//! ids: a read afterwards is a register read, never a lookup.
//!
//! [`IdentityQuad`]: lance_graph_contract::identity_quad::IdentityQuad
//!
//! # Why the text is interned during the read and sorted after
//!
//! [`IdentityCodebook`] derives an ordinal from a key's position in the
//! **sorted** key set, so ordinals cannot be handed out while reading — the set
//! is not known until the last element. Interning during the read and remapping
//! afterwards keeps one copy of each distinct string and one `u32` per
//! occurrence, instead of a `String` per occurrence.
//!
//! Measured on the Berlin extract (`tier_probe`, 2026-08-06): **2,518,465**
//! tagged features carrying **11,679,125** tag pairs — 4.64 per feature — over
//! **6,487** distinct keys and **381,362** distinct values. As
//! `(String, String)` per occurrence that is several GB of small allocations;
//! as interned pairs it is 93 MB flat, and each distinct string is stored once.
//!
//! # Spans, not per-feature vectors
//!
//! Tags live in one flat [`Vec`] and a feature carries a [`TagSpan`] into it.
//! 2.5 M individual `Vec`s would be 2.5 M allocations and 60 MB of headers
//! before a single tag is stored, and it would make [`crate::read::Feature`]
//! non-`Copy` — which the read loop relies on.
//!
//! # The capacity bound is real and it fails loudly
//!
//! A `u24` slot holds 16,777,215 distinct entries. Berlin's value codebook is
//! 381,362 — **2.3%** of that, and its key codebook 6,487 — so this extract is
//! far inside the bound. A planet-scale value set is not obviously inside it:
//! the driver is distinct `name` / `addr:housenumber` text, which grows with
//! coverage rather than saturating, and the planet is ~40× Berlin in features.
//! Berlin measuring safe is not evidence the planet does. [`IdentityCodebook::try_new`] refuses rather than
//! saturating, so the bake stops with `CodebookError::TooLarge` instead of
//! quietly aliasing two different values onto one ordinal. Splitting the value
//! space (per-key codebooks, say) is the answer if that day comes; guessing
//! that it will not is not.

use std::collections::HashMap;

use lance_graph_contract::identity_quad::{CodebookError, IdentityCodebook};

/// Assigns a provisional id to each distinct string, in first-seen order.
///
/// The id is **not** the codebook ordinal — see the module doc. It exists only
/// so the read can store a `u32` per occurrence, and is remapped by
/// [`Interner::into_codebook`].
#[derive(Debug, Default)]
pub struct Interner {
    ids: HashMap<Box<str>, u32>,
    texts: Vec<Box<str>>,
}

impl Interner {
    /// The provisional id for `s`, assigning one if this is its first sighting.
    pub fn intern(&mut self, s: &str) -> u32 {
        if let Some(&id) = self.ids.get(s) {
            return id;
        }
        let id = self.texts.len() as u32;
        let boxed: Box<str> = s.into();
        self.texts.push(boxed.clone());
        self.ids.insert(boxed, id);
        id
    }

    /// How many distinct strings have been seen.
    #[must_use]
    pub fn len(&self) -> usize {
        self.texts.len()
    }

    /// Whether nothing has been interned.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.texts.is_empty()
    }

    /// The sorted codebook, plus `remap[provisional_id] = ordinal`.
    ///
    /// # Errors
    ///
    /// [`CodebookError::TooLarge`] when the distinct set exceeds what a `u24`
    /// slot can address. The refusal is the point — see the module doc.
    pub fn into_codebook(self) -> Result<(IdentityCodebook, Vec<u32>), CodebookError> {
        let book = IdentityCodebook::try_new(self.texts.iter().map(|t| t.to_string()))?;
        // The book sorted the keys, so a provisional id and an ordinal are
        // different numbers for the same string. Ask the book rather than
        // re-deriving the sort here — re-deriving it is exactly the second
        // source of truth this remap exists to avoid.
        let mut remap = vec![0u32; self.texts.len()];
        for (id, text) in self.texts.iter().enumerate() {
            remap[id] = book
                .ordinal(text)
                .expect("every interned string was handed to the book");
        }
        Ok((book, remap))
    }
}

/// A feature's slice of the flat tag store.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct TagSpan {
    pub start: u32,
    pub len: u32,
}

impl TagSpan {
    /// Whether the feature carries no tags.
    #[must_use]
    pub const fn is_empty(self) -> bool {
        self.len == 0
    }
}

/// Every tag in the extract, interned, addressed by span.
#[derive(Debug, Default)]
pub struct TagStore {
    keys: Interner,
    values: Interner,
    pairs: Vec<(u32, u32)>,
}

impl TagStore {
    /// Intern one feature's tags and return the span that addresses them.
    pub fn push<'a>(&mut self, tags: impl Iterator<Item = (&'a str, &'a str)>) -> TagSpan {
        let start = self.pairs.len() as u32;
        for (k, v) in tags {
            let k = self.keys.intern(k);
            let v = self.values.intern(v);
            self.pairs.push((k, v));
        }
        TagSpan {
            start,
            len: self.pairs.len() as u32 - start,
        }
    }

    /// The provisional-id pairs a span addresses.
    #[must_use]
    pub fn span(&self, s: TagSpan) -> &[(u32, u32)] {
        let start = s.start as usize;
        &self.pairs[start..start + s.len as usize]
    }

    /// Distinct tag keys seen.
    #[must_use]
    pub fn distinct_keys(&self) -> usize {
        self.keys.len()
    }

    /// Distinct tag values seen.
    #[must_use]
    pub fn distinct_values(&self) -> usize {
        self.values.len()
    }

    /// Total tag pairs stored.
    #[must_use]
    pub fn pair_count(&self) -> usize {
        self.pairs.len()
    }

    /// Build both codebooks and rewrite every pair into ordinal space.
    ///
    /// # Errors
    ///
    /// [`CodebookError::TooLarge`] if either side outgrows a `u24` slot.
    pub fn resolve(mut self) -> Result<ResolvedTags, CodebookError> {
        let (keys, key_map) = std::mem::take(&mut self.keys).into_codebook()?;
        let (values, value_map) = std::mem::take(&mut self.values).into_codebook()?;
        for (k, v) in &mut self.pairs {
            *k = key_map[*k as usize];
            *v = value_map[*v as usize];
        }
        Ok(ResolvedTags {
            keys,
            values,
            pairs: self.pairs,
        })
    }
}

/// The bake's tag surface: two codebooks and every pair in ordinal space.
///
/// The codebooks are what makes an ordinal meaningful, so a bake that ships
/// rows without them ships numbers. [`IdentityCodebook::digest`] is the witness
/// that a row and a book belong to the same bake.
#[derive(Debug)]
pub struct ResolvedTags {
    pub keys: IdentityCodebook,
    pub values: IdentityCodebook,
    pairs: Vec<(u32, u32)>,
}

impl ResolvedTags {
    /// The ordinal pairs a span addresses.
    #[must_use]
    pub fn span(&self, s: TagSpan) -> &[(u32, u32)] {
        let start = s.start as usize;
        &self.pairs[start..start + s.len as usize]
    }

    /// Total pairs.
    #[must_use]
    pub fn pair_count(&self) -> usize {
        self.pairs.len()
    }

    /// Read one pair back as text — the parity check's other direction.
    #[must_use]
    pub fn text(&self, pair: (u32, u32)) -> Option<(&str, &str)> {
        Some((self.keys.key(pair.0)?, self.values.key(pair.1)?))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn interning_is_by_content_and_ids_are_stable() {
        let mut i = Interner::default();
        let a = i.intern("highway");
        let b = i.intern("name");
        assert_eq!(i.intern("highway"), a, "same text, same id");
        assert_ne!(a, b, "different text, different id");
        assert_eq!(i.len(), 2, "no duplicate stored");
    }

    #[test]
    fn the_remap_carries_provisional_ids_onto_sorted_ordinals() {
        // The load-bearing property: a provisional id is first-seen order and
        // an ordinal is sorted order, so they genuinely differ — and the remap
        // must bridge them. Interning in reverse-sorted order makes a
        // remap that quietly returned the identity fail here.
        let mut i = Interner::default();
        let z = i.intern("zebra");
        let a = i.intern("alpha");
        assert_eq!((z, a), (0, 1), "first-seen order");

        let (book, remap) = i.into_codebook().expect("small book");
        assert_eq!(remap[z as usize], book.ordinal("zebra").unwrap());
        assert_eq!(remap[a as usize], book.ordinal("alpha").unwrap());
        assert_ne!(
            remap[z as usize], z,
            "sorted order must differ from first-seen order here"
        );
        assert_eq!(book.key(remap[z as usize]), Some("zebra"));
        assert_eq!(book.key(remap[a as usize]), Some("alpha"));
    }

    #[test]
    fn tags_round_trip_through_the_store_as_text() {
        // Parity in miniature: text in, ordinals stored, the same text out.
        let mut store = TagStore::default();
        let a = store.push([("highway", "residential"), ("name", "Alexanderplatz")].into_iter());
        let b = store.push([("amenity", "cafe")].into_iter());
        let empty = store.push(std::iter::empty());

        assert_eq!(a.len, 2);
        assert_eq!(b.len, 1);
        assert!(empty.is_empty());
        assert_eq!(store.distinct_keys(), 3);
        assert_eq!(store.distinct_values(), 3);
        assert_eq!(store.pair_count(), 3);

        let resolved = store.resolve().expect("small books");
        let got: Vec<(&str, &str)> = resolved
            .span(a)
            .iter()
            .map(|&p| resolved.text(p).unwrap())
            .collect();
        assert_eq!(
            got,
            vec![("highway", "residential"), ("name", "Alexanderplatz")]
        );
        assert_eq!(
            resolved
                .span(b)
                .iter()
                .map(|&p| resolved.text(p).unwrap())
                .collect::<Vec<_>>(),
            vec![("amenity", "cafe")]
        );
        assert!(resolved.span(empty).is_empty());
    }

    #[test]
    fn a_repeated_value_under_a_different_key_stays_one_value_entry() {
        // The saving that makes the value codebook affordable, and the trap it
        // must not fall into: `building=yes` and `oneway=yes` share the VALUE
        // "yes" but not the key, so the value side must dedupe across keys
        // while the pairs stay distinct.
        let mut store = TagStore::default();
        let a = store.push([("building", "yes")].into_iter());
        let b = store.push([("oneway", "yes")].into_iter());
        assert_eq!(store.distinct_keys(), 2);
        assert_eq!(store.distinct_values(), 1, "one \"yes\"");

        let r = store.resolve().unwrap();
        assert_eq!(r.text(r.span(a)[0]), Some(("building", "yes")));
        assert_eq!(r.text(r.span(b)[0]), Some(("oneway", "yes")));
        assert_ne!(
            r.span(a)[0],
            r.span(b)[0],
            "sharing a value must not collapse the pairs"
        );
    }

    #[test]
    fn key_and_value_spaces_are_separate_books() {
        // A single shared book would make `highway` (a key) and `highway` (the
        // value of `route=highway`) one ordinal — reading a row would then be
        // unable to say which side it came from. Same text, two books, two
        // ordinal spaces that are allowed to collide numerically.
        let mut store = TagStore::default();
        let s = store.push([("highway", "bus_stop"), ("route", "highway")].into_iter());
        let r = store.resolve().unwrap();
        assert_eq!(r.keys.key(0), Some("highway"));
        assert_eq!(r.values.key(0), Some("bus_stop"));
        assert_eq!(r.text(r.span(s)[0]), Some(("highway", "bus_stop")));
        assert_eq!(r.text(r.span(s)[1]), Some(("route", "highway")));
    }
}
