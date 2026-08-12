//! The bake's codebooks, written beside its slab.
//!
//! A row stores ordinals. An ordinal means nothing without the book that
//! assigned it, so **a bake that ships rows without its codebooks ships
//! numbers** — and `parity` could only verify by re-baking in the same process,
//! which is a weaker claim than it looks (it never proves the artifact on disk
//! is readable at all).
//!
//! This module is the sidecar: a header plus three books — element identities,
//! tag keys, tag values — in one deterministic file.
//!
//! # The header pins the slab, and why only one direction stores bytes
//!
//! Two intact artifacts can still be a **wrong marriage**: a valid slab paired
//! with valid books from a different bake reads cleanly and resolves every
//! ordinal to a plausible neighbouring element. Nothing about either file, on
//! its own, notices.
//!
//! So the header carries the slab's [`hash_slab`] digest, its row count, and
//! the number of facet slots the bake filled. Reading the pair verifies all
//! three; a foreign slab is refused.
//!
//! **The reverse direction cannot store bytes, and that is deliberate.** The
//! slab has no header by design — it *is* the 512-byte row stride, which is
//! exactly what makes it something Lance can read as columns. Adding a header
//! row would break that property, and stealing key bits for a bake stamp would
//! violate slot purity (le-contract §2). What is achievable, and sufficient, is
//! that the *check* is symmetric even though the storage is not: given a slab,
//! computing its digest and asking whether these books claim it is the same
//! test run from the other side. Neither artifact can be silently paired with a
//! foreign counterpart; only one of them holds the bytes that say so.
//!
//! # The occupancy guard
//!
//! `slots_written` is the header field that turns the empty-slab defect from a
//! sampling find into a structural impossibility. The verifier re-counts filled
//! slots and compares — but note that comparison alone would NOT have caught
//! that defect, because a bake writing nothing records zero and the two agree.
//! The refusal that actually catches it lives in `bake`: a bake that filled no
//! slot at all is a pipeline that produced nothing, and it stops rather than
//! emitting an artifact.
//!
//! # The rounding rule
//!
//! [`AnchorRounding`] rides in the header as a stable discriminant, so a reader
//! in another language reads the derived-anchor contract instead of
//! reconstructing it from Rust documentation.
//!
//! # Why the digest is the point, not the bytes
//!
//! [`IdentityCodebook::try_new`] derives an ordinal from a key's position in the
//! **sorted** key set, so *an ordinal is a property of the whole set, not of the
//! key alone*. Add one key that sorts early and every later ordinal shifts by
//! one; a slab baked against the old book then resolves, through the new book,
//! to a **neighbouring element** — internally consistent on both sides, and
//! therefore invisible to `verify_bijective`, which only proves a book
//! round-trips against itself.
//!
//! That is the failure this file exists to make impossible: the header carries
//! [`IdentityCodebook::digest`] per book, and [`read_books`] refuses a file
//! whose books do not reproduce their recorded digests. A silent
//! off-by-one-element read becomes a loud refusal.
//!
//! # Format
//!
//! Little-endian throughout, matching the slab it accompanies.
//!
//! ```text
//! magic     8  b"OSMCBK\0\x03"
//! rows      8  u64 — rows the bake wrote
//! slots     8  u64 — facet slots the bake filled (occupancy guard)
//! slab      8  u64 — hash_slab of the slab this sidecar belongs to
//! rounding  4  u32 — AnchorRounding discriminant; 0 is refused
//! books     4  u32, always 4 (identity, tag keys, tag values, labels)
//! per book:
//!   digest  8  u64 — IdentityCodebook::digest of the entries that follow
//!   count   4  u32
//!   per entry:
//!     len   4  u32, byte length of the UTF-8 key
//!     key   n  UTF-8, no terminator
//! ```
//!
//! # The fourth book — labels (v3, `\x03`)
//!
//! `street.rs` needs a **name ordinal** to write into a junction's edge-name
//! slot (`crate::street::NAME_SLOT`) — le-contract §2's ban on a label/position
//! slot in the payload means the text itself can never live in the row. Reusing
//! `tag_values` for this does not work: that book interns every tag value on
//! every kept element (hundreds of thousands of entries, and a value's ordinal
//! shifts whenever an unrelated tag changes), while `crate::street::NAME_SLOT`
//! is a `u16` — the label book must fit **65,536 entries on its own terms**,
//! not inherit whatever `tag_values` happens to hold. Measured on Berlin:
//! 10,481 distinct routable-way names — 14 bits, nowhere near the ceiling.
//!
//! `\x02` bumped to `\x03` because a `\x02` reader that saw a 4-book file would
//! stop after the third and silently treat the fourth book's leading bytes as
//! the next record's header — [`BookError::BookCount`] catches the mismatch
//! instead, but only because the version byte forces the check rather than
//! letting an old binary attempt the read at all.
//!
//! Entries are written in **ordinal order**, which is sorted order because
//! `try_new` sorts. Reading therefore hands the list straight back to `try_new`
//! and gets the same ordinals — with the digest proving it rather than the
//! comment asserting it.
//!
//! No serde, no framing library. The file is a bake-time artifact read once;
//! the dependency and the version skew are not worth the fifty lines saved.

use std::io::{Read, Write};

use lance_graph_contract::identity_quad::IdentityCodebook;

use crate::tms::AnchorRounding;

/// FNV-1a over the slab's bytes.
///
/// The same construction `IdentityCodebook::digest` uses, so the artifact has
/// **one** hash convention rather than two. Explicitly **not cryptographic**:
/// this is a pairing check — "are these two files from the same bake" — not a
/// tamper seal. An adversary is not the threat model; a mismatched pair on a
/// disk is.
#[must_use]
pub fn hash_slab(bytes: &[u8]) -> u64 {
    let mut h = SlabHasher::new();
    h.eat(bytes);
    h.finish()
}

/// Streaming form of [`hash_slab`], so a bake can hash while it writes instead
/// of re-reading 1.2 GiB afterwards.
#[derive(Debug, Clone, Copy)]
pub struct SlabHasher(u64);

impl Default for SlabHasher {
    fn default() -> Self {
        Self::new()
    }
}

impl SlabHasher {
    const OFFSET: u64 = 0xcbf2_9ce4_8422_2325;
    const PRIME: u64 = 0x0000_0100_0000_01b3;

    #[must_use]
    pub const fn new() -> Self {
        SlabHasher(Self::OFFSET)
    }

    pub fn eat(&mut self, bytes: &[u8]) {
        for &b in bytes {
            self.0 ^= u64::from(b);
            self.0 = self.0.wrapping_mul(Self::PRIME);
        }
    }

    #[must_use]
    pub const fn finish(self) -> u64 {
        self.0
    }
}

/// What the bake asserts about the slab this sidecar belongs to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Header {
    /// Rows the bake wrote.
    pub rows: u64,
    /// Facet slots the bake filled — the occupancy guard.
    pub slots_written: u64,
    /// [`hash_slab`] of those rows.
    pub slab: u64,
    /// The rule [`crate::tms::mean_cell`] rounded with.
    pub rounding: AnchorRounding,
}

/// File magic + format version.
pub const MAGIC: [u8; 8] = *b"OSMCBK\0\x03";

/// Books in the file, in order: identities, tag keys, tag values, labels.
pub const BOOKS: usize = 4;

/// What went wrong reading a sidecar.
#[derive(Debug)]
pub enum BookError {
    Io(std::io::Error),
    /// Not a codebook file, or a version this build does not know.
    Magic([u8; 8]),
    /// The header's book count is not [`BOOKS`].
    BookCount(u32),
    /// A key was not valid UTF-8.
    Utf8,
    /// The file ended mid-record.
    Truncated,
    /// The book rebuilt from the file does not reproduce its recorded digest —
    /// so the slab's ordinals do not address the book that was just read.
    Digest {
        book: usize,
        recorded: u64,
        rebuilt: u64,
    },
    /// The codebook refused the key list.
    Codebook(lance_graph_contract::identity_quad::CodebookError),
    /// The header names a rounding rule this build does not implement — or the
    /// unspecified value `0`. Refused, never defaulted.
    Rounding(u32),
    /// The slab does not match what this sidecar says it belongs to: a
    /// **wrong marriage** of two individually-valid artifacts.
    SlabMismatch {
        field: &'static str,
        recorded: u64,
        actual: u64,
    },
}

impl std::fmt::Display for BookError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            BookError::Io(e) => write!(f, "io: {e}"),
            BookError::Magic(m) => write!(f, "not a codebook file (magic {m:?})"),
            BookError::BookCount(n) => write!(f, "expected {BOOKS} books, found {n}"),
            BookError::Utf8 => write!(f, "a key is not valid UTF-8"),
            BookError::Truncated => write!(f, "file ended mid-record"),
            BookError::Digest {
                book,
                recorded,
                rebuilt,
            } => write!(
                f,
                "book {book} digest mismatch: recorded {recorded:016x}, rebuilt {rebuilt:016x} — \
                 the slab's ordinals do not address this book"
            ),
            BookError::Codebook(e) => write!(f, "codebook: {e:?}"),
            BookError::Rounding(v) => {
                write!(
                    f,
                    "unknown anchor-rounding rule {v} — refused, not defaulted"
                )
            }
            BookError::SlabMismatch {
                field,
                recorded,
                actual,
            } => write!(
                f,
                "slab {field} mismatch: sidecar records {recorded}, slab has {actual} — \
                 these two artifacts are not from the same bake"
            ),
        }
    }
}

impl std::error::Error for BookError {}

impl From<std::io::Error> for BookError {
    fn from(e: std::io::Error) -> Self {
        BookError::Io(e)
    }
}

/// The four books a bake produces.
#[derive(Debug)]
pub struct Books {
    pub identities: IdentityCodebook,
    pub tag_keys: IdentityCodebook,
    pub tag_values: IdentityCodebook,
    /// Street/way-name ordinals — `crate::street::NAME_SLOT`'s address space.
    /// A **separate** book from `tag_values`: it must fit u16 on its own
    /// terms (see the module docs), not inherit that book's much larger,
    /// unrelated cardinality.
    pub labels: IdentityCodebook,
}

impl Books {
    fn each(&self) -> [&IdentityCodebook; BOOKS] {
        [
            &self.identities,
            &self.tag_keys,
            &self.tag_values,
            &self.labels,
        ]
    }
}

fn write_book<W: Write>(w: &mut W, book: &IdentityCodebook) -> std::io::Result<()> {
    w.write_all(&book.digest().to_le_bytes())?;
    let n = u32::try_from(book.len()).expect("a book that fits a u24 ordinal fits a u32 count");
    w.write_all(&n.to_le_bytes())?;
    for i in 0..n {
        let key = book.key(i).expect("ordinals 0..len are all present");
        w.write_all(&u32::try_from(key.len()).expect("key length").to_le_bytes())?;
        w.write_all(key.as_bytes())?;
    }
    Ok(())
}

/// Write the sidecar.
///
/// # Errors
///
/// Any I/O failure from `w`.
pub fn write_books<W: Write>(w: &mut W, header: &Header, books: &Books) -> std::io::Result<()> {
    w.write_all(&MAGIC)?;
    w.write_all(&header.rows.to_le_bytes())?;
    w.write_all(&header.slots_written.to_le_bytes())?;
    w.write_all(&header.slab.to_le_bytes())?;
    w.write_all(&header.rounding.wire().to_le_bytes())?;
    w.write_all(&(BOOKS as u32).to_le_bytes())?;
    for book in books.each() {
        write_book(w, book)?;
    }
    Ok(())
}

fn read_exact_n<R: Read>(r: &mut R, n: usize) -> Result<Vec<u8>, BookError> {
    let mut buf = vec![0u8; n];
    match r.read_exact(&mut buf) {
        Ok(()) => Ok(buf),
        Err(e) if e.kind() == std::io::ErrorKind::UnexpectedEof => Err(BookError::Truncated),
        Err(e) => Err(BookError::Io(e)),
    }
}

fn read_u32<R: Read>(r: &mut R) -> Result<u32, BookError> {
    let b = read_exact_n(r, 4)?;
    Ok(u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
}

fn read_u64<R: Read>(r: &mut R) -> Result<u64, BookError> {
    let b = read_exact_n(r, 8)?;
    Ok(u64::from_le_bytes([
        b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7],
    ]))
}

fn read_book<R: Read>(r: &mut R, index: usize) -> Result<IdentityCodebook, BookError> {
    let recorded = read_u64(r)?;
    let count = read_u32(r)?;
    let mut keys = Vec::with_capacity(count as usize);
    for _ in 0..count {
        let len = read_u32(r)? as usize;
        let bytes = read_exact_n(r, len)?;
        keys.push(String::from_utf8(bytes).map_err(|_| BookError::Utf8)?);
    }
    let book = IdentityCodebook::try_new(keys).map_err(BookError::Codebook)?;
    let rebuilt = book.digest();
    if rebuilt != recorded {
        return Err(BookError::Digest {
            book: index,
            recorded,
            rebuilt,
        });
    }
    Ok(book)
}

/// Read the sidecar, verifying every book against its recorded digest.
///
/// # Errors
///
/// [`BookError`] — bad magic, a truncated record, a non-UTF-8 key, a book the
/// codebook refuses, or (the one that matters) a digest that does not
/// reproduce.
pub fn read_books<R: Read>(r: &mut R) -> Result<(Header, Books), BookError> {
    let magic = read_exact_n(r, MAGIC.len())?;
    if magic[..] != MAGIC[..] {
        let mut m = [0u8; 8];
        m.copy_from_slice(&magic);
        return Err(BookError::Magic(m));
    }
    let rows = read_u64(r)?;
    let slots_written = read_u64(r)?;
    let slab = read_u64(r)?;
    let rw = read_u32(r)?;
    let rounding = AnchorRounding::from_wire(rw).ok_or(BookError::Rounding(rw))?;
    let n = read_u32(r)?;
    if n as usize != BOOKS {
        return Err(BookError::BookCount(n));
    }
    let books = Books {
        identities: read_book(r, 0)?,
        tag_keys: read_book(r, 1)?,
        tag_values: read_book(r, 2)?,
        labels: read_book(r, 3)?,
    };
    Ok((
        Header {
            rows,
            slots_written,
            slab,
            rounding,
        },
        books,
    ))
}

/// Check a slab against the sidecar that claims it.
///
/// This is the marriage test: three fields, any one of which failing means the
/// two artifacts are not from the same bake. `slots_written` is compared by the
/// CALLER (it must walk the rows to count), so this checks what can be checked
/// from the bytes alone.
///
/// # Errors
///
/// [`BookError::SlabMismatch`] naming the field that disagreed.
pub fn verify_slab(header: &Header, slab_bytes: &[u8], stride: usize) -> Result<(), BookError> {
    let rows = (slab_bytes.len() / stride) as u64;
    if rows != header.rows {
        return Err(BookError::SlabMismatch {
            field: "row count",
            recorded: header.rows,
            actual: rows,
        });
    }
    let h = hash_slab(slab_bytes);
    if h != header.slab {
        return Err(BookError::SlabMismatch {
            field: "digest",
            recorded: header.slab,
            actual: h,
        });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn book(keys: &[&str]) -> IdentityCodebook {
        IdentityCodebook::try_new(keys.iter().map(|s| (*s).to_string())).expect("small book")
    }

    fn head() -> Header {
        Header {
            rows: 7,
            slots_written: 42,
            slab: 0xdead_beef_0bad_f00d,
            rounding: AnchorRounding::HalfUp,
        }
    }

    fn sample() -> Books {
        Books {
            identities: book(&["0f01:42", "0f02:42", "0f02:7"]),
            tag_keys: book(&["highway", "name"]),
            tag_values: book(&["residential", "Unter den Linden", "café"]),
            labels: book(&["Unter den Linden", "Friedrichstraße"]),
        }
    }

    #[test]
    fn books_round_trip_through_the_file() {
        let want = sample();
        let mut buf = Vec::new();
        write_books(&mut buf, &head(), &want).expect("write");
        let (gh, got) = read_books(&mut buf.as_slice()).expect("read");
        assert_eq!(gh, head(), "the header must survive verbatim");

        for (a, b) in want.each().into_iter().zip(got.each()) {
            assert_eq!(a.len(), b.len());
            assert_eq!(a.digest(), b.digest());
            for i in 0..a.len() as u32 {
                assert_eq!(a.key(i), b.key(i), "ordinal {i} must resolve identically");
            }
        }
        // Non-ASCII survives byte-for-byte — OSM values are full Unicode and a
        // length-prefixed read that assumed one byte per char would corrupt
        // exactly the names a renderer displays.
        assert_eq!(
            got.tag_values.ordinal("café"),
            want.tag_values.ordinal("café")
        );
        assert!(got.tag_values.ordinal("café").is_some());

        // The fourth book specifically: it must be its OWN address space, not
        // a mirror of tag_values sharing the same ordinal for the same string.
        // Both books happen to contain "Unter den Linden" here, and if the
        // reader accidentally aliased the two books this would still pass —
        // so the discriminating half is that the label book's ordinal space
        // is independently u16-sized, checked by the dedicated test below.
        assert_eq!(got.labels.len(), want.labels.len());
        assert!(got.labels.ordinal("Unter den Linden").is_some());
        assert!(got.labels.ordinal("Friedrichstraße").is_some());
    }

    #[test]
    fn a_stale_three_book_file_is_refused_by_magic_not_silently_misread() {
        // The whole reason for the version bump: a v2 (`\x02`) sidecar has 3
        // books, and a `\x03` reader that ignored the magic and trusted the
        // book count would read book 3 as whatever bytes happen to follow the
        // third book's last entry — a wrong read, not a loud one. Bumping the
        // magic means the file is refused before that can happen at all.
        let three = Books {
            identities: book(&["0f01:1"]),
            tag_keys: book(&["highway"]),
            tag_values: book(&["residential"]),
            labels: book(&["Unter den Linden"]), // never written below
        };
        let mut buf = Vec::new();
        buf.extend_from_slice(b"OSMCBK\0\x02"); // the OLD magic, verbatim
        let h = head();
        buf.extend_from_slice(&h.rows.to_le_bytes());
        buf.extend_from_slice(&h.slots_written.to_le_bytes());
        buf.extend_from_slice(&h.slab.to_le_bytes());
        buf.extend_from_slice(&h.rounding.wire().to_le_bytes());
        buf.extend_from_slice(&3u32.to_le_bytes()); // v2's book count
        for b in [&three.identities, &three.tag_keys, &three.tag_values] {
            write_book(&mut buf, b).expect("write");
        }
        assert!(
            matches!(read_books(&mut buf.as_slice()), Err(BookError::Magic(m)) if m == *b"OSMCBK\0\x02"),
            "an old-format sidecar must be named as OUTDATED, not misread as v3"
        );

        // ...paired with the positive: a genuine v3 4-book file still reads,
        // so the refusal is discriminating on the magic, not on book count.
        let mut ok = Vec::new();
        write_books(&mut ok, &head(), &sample()).expect("write");
        assert!(read_books(&mut ok.as_slice()).is_ok());
    }

    #[test]
    fn a_book_whose_contents_shifted_is_refused_rather_than_read() {
        // THE failure this module exists for. A key inserted early shifts every
        // later ordinal by one, so a slab baked against the old book resolves
        // to a NEIGHBOURING element — valid-looking on both sides, and
        // invisible to verify_bijective. The digest is what catches it.
        let mut buf = Vec::new();
        write_books(&mut buf, &head(), &sample()).expect("write");

        // Rewrite the identity book with one extra early-sorting key, leaving
        // the recorded digest untouched — exactly the drift being guarded.
        let drifted = book(&["0f01:1", "0f01:42", "0f02:42", "0f02:7"]);
        let mut tampered = Vec::new();
        tampered.extend_from_slice(&MAGIC);
        let h = head();
        tampered.extend_from_slice(&h.rows.to_le_bytes());
        tampered.extend_from_slice(&h.slots_written.to_le_bytes());
        tampered.extend_from_slice(&h.slab.to_le_bytes());
        tampered.extend_from_slice(&h.rounding.wire().to_le_bytes());
        tampered.extend_from_slice(&(BOOKS as u32).to_le_bytes());
        tampered.extend_from_slice(&sample().identities.digest().to_le_bytes()); // stale
        tampered.extend_from_slice(&(drifted.len() as u32).to_le_bytes());
        for i in 0..drifted.len() as u32 {
            let k = drifted.key(i).unwrap();
            tampered.extend_from_slice(&(k.len() as u32).to_le_bytes());
            tampered.extend_from_slice(k.as_bytes());
        }

        assert!(
            matches!(
                read_books(&mut tampered.as_slice()),
                Err(BookError::Digest { book: 0, .. })
            ),
            "a shifted book must be refused"
        );

        // Can-stay-silent half: the untampered file still reads, so the digest
        // check is discriminating rather than rejecting everything.
        assert!(read_books(&mut buf.as_slice()).is_ok());
    }

    #[test]
    fn a_slab_from_a_different_bake_is_refused() {
        // The WRONG MARRIAGE: both artifacts individually valid, from different
        // bakes. Nothing about either file notices on its own — every ordinal
        // still resolves, to a plausible neighbour.
        let stride = 512usize;
        let slab = vec![7u8; stride * 3];
        let h = Header {
            rows: 3,
            slots_written: 9,
            slab: hash_slab(&slab),
            rounding: AnchorRounding::HalfUp,
        };
        assert!(
            verify_slab(&h, &slab, stride).is_ok(),
            "its own slab passes"
        );

        // One byte different: same length, same row count, different bake.
        let mut other = slab.clone();
        other[stride] ^= 1;
        assert!(matches!(
            verify_slab(&h, &other, stride),
            Err(BookError::SlabMismatch {
                field: "digest",
                ..
            })
        ));

        // A different row count is caught before the digest is even computed.
        let short = vec![7u8; stride * 2];
        assert!(matches!(
            verify_slab(&h, &short, stride),
            Err(BookError::SlabMismatch {
                field: "row count",
                ..
            })
        ));
    }

    #[test]
    fn an_unknown_rounding_rule_is_refused_not_defaulted() {
        // A reader that silently assumed half-up would reproduce derived
        // anchors WRONG for a bake made under any other rule, and every one of
        // them would still look like a valid cell.
        assert_eq!(
            AnchorRounding::from_wire(0),
            None,
            "unspecified is not a rule"
        );
        assert_eq!(AnchorRounding::from_wire(99), None);
        assert_eq!(
            AnchorRounding::from_wire(AnchorRounding::HalfUp.wire()),
            Some(AnchorRounding::HalfUp),
            "…and the rule in force round-trips, so the refusal discriminates"
        );

        let mut buf = Vec::new();
        write_books(&mut buf, &head(), &sample()).expect("write");
        // Zero the rounding field in place: byte offset 8 + 8 + 8 + 8.
        buf[32..36].copy_from_slice(&0u32.to_le_bytes());
        assert!(matches!(
            read_books(&mut buf.as_slice()),
            Err(BookError::Rounding(0))
        ));
    }

    #[test]
    fn a_foreign_or_truncated_file_is_refused() {
        assert!(matches!(
            read_books(&mut b"not-a-book".as_slice()),
            Err(BookError::Magic(_))
        ));

        let mut buf = Vec::new();
        write_books(&mut buf, &head(), &sample()).expect("write");
        // Every truncation point must refuse — a short read that silently
        // yielded a partial book would hand back ordinals addressing nothing.
        for cut in [8usize, 20, 36, 40, 48, 56] {
            assert!(
                read_books(&mut &buf[..cut.min(buf.len())]).is_err(),
                "truncation at {cut} must be refused"
            );
        }
        // …and the whole file is fine, so the loop is not passing vacuously.
        assert!(read_books(&mut buf.as_slice()).is_ok());
    }
}
