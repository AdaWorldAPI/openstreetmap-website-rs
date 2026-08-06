//! The bake's codebooks, written beside its slab.
//!
//! A row stores ordinals. An ordinal means nothing without the book that
//! assigned it, so **a bake that ships rows without its codebooks ships
//! numbers** — and `parity` could only verify by re-baking in the same process,
//! which is a weaker claim than it looks (it never proves the artifact on disk
//! is readable at all).
//!
//! This module is the sidecar: three books — element identities, tag keys, tag
//! values — in one deterministic file.
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
//! magic     8  b"OSMCBK\0\x01"
//! books     4  u32, always 3 (identity, tag keys, tag values)
//! per book:
//!   digest  8  u64 — IdentityCodebook::digest of the entries that follow
//!   count   4  u32
//!   per entry:
//!     len   4  u32, byte length of the UTF-8 key
//!     key   n  UTF-8, no terminator
//! ```
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

/// File magic + format version.
pub const MAGIC: [u8; 8] = *b"OSMCBK\0\x01";

/// Books in the file, in order: identities, tag keys, tag values.
pub const BOOKS: usize = 3;

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
        }
    }
}

impl std::error::Error for BookError {}

impl From<std::io::Error> for BookError {
    fn from(e: std::io::Error) -> Self {
        BookError::Io(e)
    }
}

/// The three books a bake produces.
#[derive(Debug)]
pub struct Books {
    pub identities: IdentityCodebook,
    pub tag_keys: IdentityCodebook,
    pub tag_values: IdentityCodebook,
}

impl Books {
    fn each(&self) -> [&IdentityCodebook; BOOKS] {
        [&self.identities, &self.tag_keys, &self.tag_values]
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
pub fn write_books<W: Write>(w: &mut W, books: &Books) -> std::io::Result<()> {
    w.write_all(&MAGIC)?;
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
pub fn read_books<R: Read>(r: &mut R) -> Result<Books, BookError> {
    let magic = read_exact_n(r, MAGIC.len())?;
    if magic[..] != MAGIC[..] {
        let mut m = [0u8; 8];
        m.copy_from_slice(&magic);
        return Err(BookError::Magic(m));
    }
    let n = read_u32(r)?;
    if n as usize != BOOKS {
        return Err(BookError::BookCount(n));
    }
    Ok(Books {
        identities: read_book(r, 0)?,
        tag_keys: read_book(r, 1)?,
        tag_values: read_book(r, 2)?,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn book(keys: &[&str]) -> IdentityCodebook {
        IdentityCodebook::try_new(keys.iter().map(|s| (*s).to_string())).expect("small book")
    }

    fn sample() -> Books {
        Books {
            identities: book(&["0f01:42", "0f02:42", "0f02:7"]),
            tag_keys: book(&["highway", "name"]),
            tag_values: book(&["residential", "Unter den Linden", "café"]),
        }
    }

    #[test]
    fn books_round_trip_through_the_file() {
        let want = sample();
        let mut buf = Vec::new();
        write_books(&mut buf, &want).expect("write");
        let got = read_books(&mut buf.as_slice()).expect("read");

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
    }

    #[test]
    fn a_book_whose_contents_shifted_is_refused_rather_than_read() {
        // THE failure this module exists for. A key inserted early shifts every
        // later ordinal by one, so a slab baked against the old book resolves
        // to a NEIGHBOURING element — valid-looking on both sides, and
        // invisible to verify_bijective. The digest is what catches it.
        let mut buf = Vec::new();
        write_books(&mut buf, &sample()).expect("write");

        // Rewrite the identity book with one extra early-sorting key, leaving
        // the recorded digest untouched — exactly the drift being guarded.
        let drifted = book(&["0f01:1", "0f01:42", "0f02:42", "0f02:7"]);
        let mut tampered = Vec::new();
        tampered.extend_from_slice(&MAGIC);
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
    fn a_foreign_or_truncated_file_is_refused() {
        assert!(matches!(
            read_books(&mut b"not-a-book".as_slice()),
            Err(BookError::Magic(_))
        ));

        let mut buf = Vec::new();
        write_books(&mut buf, &sample()).expect("write");
        // Every truncation point must refuse — a short read that silently
        // yielded a partial book would hand back ordinals addressing nothing.
        for cut in [8usize, 12, 20, 24, 30] {
            assert!(
                read_books(&mut &buf[..cut.min(buf.len())]).is_err(),
                "truncation at {cut} must be refused"
            );
        }
        // …and the whole file is fine, so the loop is not passing vacuously.
        assert!(read_books(&mut buf.as_slice()).is_ok());
    }
}
