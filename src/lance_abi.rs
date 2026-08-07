//! The bake as a **Lance dataset** — hydration and versioning, never a query.
//!
//! `bake.rs` has said since it was written that *"wrapping it in a Lance
//! dataset is the next step and changes no byte of this file."* This module is
//! that step, and the sentence is literally true: the same bytes, in a table.
//!
//! # The pattern is MedCare-rs's, not a new one
//!
//! `medcare-soa::lance_io::hydrate_node_rows` writes a `.soa` crystal into
//! Lance as **one `FixedSizeBinary(512)` column** — the row is not decomposed
//! into typed fields. Its reasoning applies here unchanged and is worth
//! restating because it is the whole design:
//!
//! > *"The key already addresses the node … Splitting the row into typed
//! > columns would move that interpretation into the WRITER's schema — freezing
//! > today's reading of a deliberately content-blind register, and forcing a
//! > migration every time a ClassView learns a new projection over the same
//! > bytes. Opaque storage keeps the substrate's rule intact: the bytes hold
//! > nothing, the view reads."*
//!
//! A geo row is exactly that case: [`crate::project`] reads it through a
//! `WideFieldMask`, and `ogar_osm::GEO_V3_FACET` says what each mask position
//! means. A typed Lance schema would be a **second** answer to that question.
//!
//! # What zero-copy means here, concretely
//!
//! Arrow stores a `FixedSizeBinary(512)` column as ONE contiguous buffer of
//! `n * 512` bytes. That is bit-for-bit what [`RowSlab`] already wraps — so
//! [`slab_of`] hands back a slab **pointing into the Arrow buffer**. No decode,
//! no gather, no `Vec`. [`tests::the_slab_points_into_the_arrow_buffer`] asserts
//! the pointers are equal, which is the only way to tell a borrow from a
//! well-optimised copy.
//!
//! **This holds on the Lance read path too — but it did not at first, and the
//! repair is the load-bearing part.** A batch read back from a real dataset is
//! 8-byte aligned, and whether it is *also* 64-aligned varies with allocator
//! state. `NodeRow` is `align(64)`, so the first version REFUSED the borrow and
//! offered a copy instead. That was the violation: zero copy is a law without
//! escape hatches, and a guard whose only lawful alternative is a copy has not
//! protected anything.
//!
//! The check was simply in the wrong place. `RowSlab::morton_at` reads BYTES at
//! a computed offset, so `lower_bound` and `tile_range` never needed the
//! `&[NodeRow]` cast — alignment now gates only `RowSlab::rows()`, the one
//! operation that actually casts, and the entire lookup path borrows at any
//! alignment. Pinned by
//! [`tests::the_lance_read_path_is_borrowed_at_any_alignment`].
//!
//! MedCare's own round-trip test copies the rows back out
//! (`back.extend_from_slice(col.value(i))`) because it is checking *fidelity*.
//! That is the right test for its question and the wrong shape for this one:
//! the pointer identity is what this module owes.
//!
//! # Lance as FORMAT, not as ENGINE
//!
//! A tile is still resolved by [`RowSlab::tile_range`] — Morton prefix
//! arithmetic over the borrowed rows. Lance supplies durability, versioning and
//! the S3 hydration path; it does not plan the lookup. The two jobs are
//! separable precisely because the row is opaque to it.
//!
//! # Feature-gated
//!
//! Behind `lance`, so the default build of this crate keeps the dependency set
//! its manifest declares (`osmpbf` + `lance-graph-contract` + `ogar-*`) and the
//! `bake` binary stays free of the lancedb tree.

use arrow::array::{Array, FixedSizeBinaryArray};
use arrow::datatypes::{DataType, Field, Schema};
use arrow::record_batch::RecordBatch;
use futures::TryStreamExt;
use lancedb::query::ExecutableQuery;
use lancedb::Connection;
use std::sync::Arc;

use crate::slab::{RowSlab, SlabError};
use lance_graph_contract::canonical_node::NODE_ROW_STRIDE;

/// The column name every geo bake table uses.
///
/// One name, declared once: a reader that guesses `"rows"` against a writer
/// that wrote `"row"` fails at `column_by_name`, which is a worse error than a
/// type mismatch because it reads as "empty table".
pub const ROW_COLUMN: &str = "row";

/// Why a batch could not be viewed as a slab.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AbiError {
    /// The batch has no column by [`ROW_COLUMN`].
    NoRowColumn,
    /// The column is not `FixedSizeBinary(512)`.
    WrongType {
        /// What the column actually is.
        got: String,
    },
    /// The buffer is not row-aligned or not 64-byte aligned.
    Slab(SlabError),
}

impl core::fmt::Display for AbiError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::NoRowColumn => write!(f, "batch has no `{ROW_COLUMN}` column"),
            Self::WrongType { got } => {
                write!(
                    f,
                    "`{ROW_COLUMN}` is {got}, not FixedSizeBinary({NODE_ROW_STRIDE})"
                )
            }
            Self::Slab(e) => write!(f, "buffer is not a valid slab: {e:?}"),
        }
    }
}

impl std::error::Error for AbiError {}

/// The Arrow schema a geo bake table carries: one opaque row column.
#[must_use]
pub fn row_schema() -> Arc<Schema> {
    Arc::new(Schema::new(vec![Field::new(
        ROW_COLUMN,
        DataType::FixedSizeBinary(NODE_ROW_STRIDE as i32),
        false,
    )]))
}

/// Open — or create — a Lance store at `uri`.
///
/// `uri` is a directory path or an object-store URL. A missing local directory
/// is created on first write, so a fresh path is a valid empty store.
///
/// # Errors
/// Propagates any `lancedb` connect failure.
pub async fn connect(uri: &str) -> lancedb::Result<Connection> {
    lancedb::connect(uri).execute().await
}

/// Write a `.soa` byte run into `table` as opaque `FixedSizeBinary(512)` rows.
///
/// **Appends.** Hydrating twice doubles the rows — [`table_rows`] is the cheap
/// guard, and the caller owns it, exactly as in MedCare's version. A bake's row
/// count is known from its codebook header, which is what makes the check exact
/// rather than a content comparison.
///
/// # Errors
/// [`lancedb::Error::InvalidInput`] if `bytes` is not a whole number of
/// 512-byte rows — a truncated artifact must not become a short table that
/// looks like a successful hydrate. Otherwise propagates Lance failures.
pub async fn hydrate(conn: &Connection, table: &str, bytes: &[u8]) -> lancedb::Result<usize> {
    if bytes.is_empty() || !bytes.len().is_multiple_of(NODE_ROW_STRIDE) {
        return Err(lancedb::Error::InvalidInput {
            message: format!(
                "slab is {} bytes, not a whole number of {NODE_ROW_STRIDE}-byte rows — \
                 refusing to hydrate a truncated bake",
                bytes.len()
            ),
        });
    }
    let rows = bytes.len() / NODE_ROW_STRIDE;
    let array =
        FixedSizeBinaryArray::try_from_iter(bytes.chunks_exact(NODE_ROW_STRIDE)).map_err(|e| {
            lancedb::Error::InvalidInput {
                message: format!("build FixedSizeBinary({NODE_ROW_STRIDE}) from slab: {e}"),
            }
        })?;
    let batch = RecordBatch::try_new(row_schema(), vec![Arc::new(array)]).map_err(|e| {
        lancedb::Error::InvalidInput {
            message: format!("build RecordBatch for {rows} rows: {e}"),
        }
    })?;

    let names = conn.table_names().execute().await?;
    if names.iter().any(|t| t == table) {
        let tbl = conn.open_table(table).execute().await?;
        tbl.add(batch).execute().await?;
    } else {
        conn.create_table(table, batch).execute().await?;
    }
    Ok(rows)
}

/// How many rows `table` holds — `None` when it does not exist.
///
/// The cheap half of the hydrate gate.
///
/// # Errors
/// Propagates any `lancedb` list / open / count failure.
pub async fn table_rows(conn: &Connection, table: &str) -> lancedb::Result<Option<usize>> {
    if !conn
        .table_names()
        .execute()
        .await?
        .iter()
        .any(|t| t == table)
    {
        return Ok(None);
    }
    let tbl = conn.open_table(table).execute().await?;
    Ok(Some(tbl.count_rows(None).await?))
}

/// Read `table` back as ONE batch, or `None` when it holds no rows.
///
/// Lance may return several fragments; they are concatenated so the caller gets
/// a single contiguous buffer to slab. **A concat COPIES** — which is why a
/// single-fragment table is the shape this path wants, and why [`slab_of`]
/// exists separately so a caller who reads fragment-at-a-time can slab each one
/// without ever concatenating.
///
/// # Errors
/// Propagates any `lancedb` read failure, or an `arrow` concat error.
pub async fn read_batch(conn: &Connection, table: &str) -> lancedb::Result<Option<RecordBatch>> {
    let tbl = conn.open_table(table).execute().await?;
    let batches: Vec<RecordBatch> = tbl
        .query()
        .execute()
        .await?
        .try_collect::<Vec<RecordBatch>>()
        .await?;
    if batches.is_empty() {
        return Ok(None);
    }
    if batches.len() == 1 {
        return Ok(batches.into_iter().next());
    }
    let schema = batches[0].schema();
    arrow::compute::concat_batches(&schema, &batches)
        .map(Some)
        .map_err(|e| lancedb::Error::Other {
            message: format!("concat {} fragments: {e}", batches.len()),
            source: None,
        })
}

/// **The pointer.** View a batch's row column as a [`RowSlab`] — borrowed,
/// pointing into Arrow's own buffer, copying nothing.
///
/// This is what "zero copy" means at this seam: the bytes Lance read from disk
/// or S3 are the bytes the slab addresses, and `tile_range` does its Morton
/// arithmetic on them in place.
///
/// # Errors
/// [`AbiError`] when the column is missing, mistyped, sliced, or the buffer
/// fails the slab's stride/alignment check. Every one of those is a refusal
/// rather than a silent reinterpretation.
pub fn slab_of(batch: &RecordBatch) -> Result<RowSlab<'_>, AbiError> {
    let col = batch
        .column_by_name(ROW_COLUMN)
        .ok_or(AbiError::NoRowColumn)?;
    let fsb = col
        .as_any()
        .downcast_ref::<FixedSizeBinaryArray>()
        .ok_or_else(|| AbiError::WrongType {
            got: format!("{}", col.data_type()),
        })?;
    if fsb.value_length() != NODE_ROW_STRIDE as i32 {
        return Err(AbiError::WrongType {
            got: format!("FixedSizeBinary({})", fsb.value_length()),
        });
    }
    // NO slice guard here, and that is a MEASURED decision rather than an
    // oversight. An earlier version refused `offset() != 0`, reasoning that a
    // sliced array's `value_data()` still starts at the parent's origin. Arrow
    // does not work that way: both `RecordBatch::slice(2, 3)` and an
    // `ArrayData` built with an explicit `.offset(2)` report `offset() == 0`
    // and a REBASED `value_data()` (pinned by
    // `what_a_slice_does_to_the_buffer_decides_whether_the_guard_is_needed`).
    // The guard could not fire, and a guard that cannot fire carries exactly as
    // much information as one that always does.
    let buf = fsb.value_data();
    // `value_data` may run past this array's own rows when the buffer is
    // shared; trim to exactly the rows this array claims.
    let want = fsb.len() * NODE_ROW_STRIDE;
    let bytes = buf.get(..want).unwrap_or(buf);
    RowSlab::new(bytes).map_err(AbiError::Slab)
}

#[cfg(test)]
mod tests {
    use super::*;
    use lance_graph_contract::canonical_node::NodeRow;

    /// `n` rows whose first key byte is the row index, so a misread is visible.
    fn slab_bytes(n: usize) -> Vec<u8> {
        let mut v = vec![0u8; n * NODE_ROW_STRIDE];
        for i in 0..n {
            v[i * NODE_ROW_STRIDE] = (i + 1) as u8;
        }
        v
    }

    fn batch_of(bytes: &[u8]) -> RecordBatch {
        let a = FixedSizeBinaryArray::try_from_iter(bytes.chunks_exact(NODE_ROW_STRIDE))
            .expect("build array");
        RecordBatch::try_new(row_schema(), vec![Arc::new(a)]).expect("build batch")
    }

    /// **The claim of this module.** The slab must POINT INTO Arrow's buffer.
    ///
    /// Comparing contents would pass for a copy too — an optimised
    /// `to_vec()` produces byte-identical output. Only pointer equality
    /// distinguishes a borrow from a copy, which is exactly what "zero copy
    /// via pointer" asserts.
    #[test]
    fn the_slab_points_into_the_arrow_buffer() {
        let bytes = slab_bytes(4);
        let batch = batch_of(&bytes);
        let col = batch
            .column_by_name(ROW_COLUMN)
            .unwrap()
            .as_any()
            .downcast_ref::<FixedSizeBinaryArray>()
            .unwrap();
        let arrow_ptr = col.value_data().as_ptr();

        let slab = slab_of(&batch).expect("slab");
        let slab_ptr = slab.rows().expect("aligned").as_ptr().cast::<u8>();

        assert_eq!(
            arrow_ptr, slab_ptr,
            "the slab must borrow Arrow's buffer, not copy it"
        );
        assert_eq!(slab.len(), 4);
    }

    /// The paired silence half: a slab built from a COPY of the same bytes has
    /// the same contents and a DIFFERENT pointer. Without this, the test above
    /// could pass by accident on an allocator that happened to reuse an address.
    #[test]
    fn a_copy_has_the_same_bytes_and_a_different_pointer() {
        let bytes = slab_bytes(4);
        let batch = batch_of(&bytes);
        let col = batch
            .column_by_name(ROW_COLUMN)
            .unwrap()
            .as_any()
            .downcast_ref::<FixedSizeBinaryArray>()
            .unwrap();

        let copied: Vec<u8> = col.value_data().to_vec();
        assert_eq!(
            copied, bytes,
            "a copy is byte-identical — which is the point"
        );
        assert_ne!(
            copied.as_ptr(),
            col.value_data().as_ptr(),
            "…and yet it is a different buffer, which only the pointer shows"
        );
    }

    /// The rows read back must be the rows written, in order — pointer identity
    /// alone would pass on a buffer that is aliased but misaligned by a row.
    #[test]
    fn the_rows_are_the_rows_that_were_written_in_order() {
        let bytes = slab_bytes(5);
        let batch = batch_of(&bytes);
        let slab = slab_of(&batch).expect("slab");
        let rows: &[NodeRow] = slab.rows().expect("aligned");
        assert_eq!(rows.len(), 5);
        for (i, r) in rows.iter().enumerate() {
            assert_eq!(
                r.key.as_bytes()[0],
                (i + 1) as u8,
                "row {i} is not the row that was written at index {i}"
            );
        }
    }

    /// **What `slice` actually does to the buffer — measured, then pinned.**
    ///
    /// The first version of this test *assumed* a sliced array's `value_data()`
    /// still starts at the parent's origin, asserted `offset() == 2`, and
    /// FAILED with `offset() == 0`. That failure is the finding, and it killed
    /// a guard: arrow **rebases** the buffer, so slabbing a slice is safe and
    /// the offset check `slab_of` used to carry could never fire.
    ///
    /// Pinned two-sidedly here so the guard is not re-added on intuition: the
    /// slice must actually move the window (`value(0)` is the third row), AND
    /// the raw buffer must move with it (`value_data()[0]` agrees). If a future
    /// arrow ever shares the parent's buffer again, the second assertion goes
    /// red and the guard's removal is re-opened — which is exactly when someone
    /// should look.
    #[test]
    fn what_a_slice_does_to_the_buffer_decides_whether_the_guard_is_needed() {
        let bytes = slab_bytes(6);
        let batch = batch_of(&bytes);
        let sliced = batch.slice(2, 3);
        let col = sliced
            .column_by_name(ROW_COLUMN)
            .unwrap()
            .as_any()
            .downcast_ref::<FixedSizeBinaryArray>()
            .unwrap();

        assert_eq!(
            col.value(0)[0],
            3,
            "value(0) must be the third row — the slice doing its job"
        );
        assert_eq!(
            col.value_data()[0],
            3,
            "…and the RAW buffer must agree: arrow rebased it, so there is no \
             parent-origin hazard to guard against"
        );
        assert_eq!(col.offset(), 0, "a rebased buffer reports no offset");

        // Therefore the slice slabs cleanly, at the requested row.
        let slab = slab_of(&sliced).expect("a rebased slice must slab cleanly");
        assert_eq!(slab.len(), 3);
        assert_eq!(
            slab.rows().expect("aligned")[0].key.as_bytes()[0],
            3,
            "the slab starts at the sliced row, not at the parent's row 0"
        );
    }

    /// **The round trip through a REAL Lance dataset** — the bytes survive.
    ///
    /// Copy-based on purpose: this asserts *fidelity*, and fidelity is the
    /// claim a copy can carry. Whether the returned buffer can be BORROWED as
    /// rows is a separate question with a separate, currently-negative answer —
    /// see [`the_lance_read_path_is_not_yet_64_byte_aligned`].
    #[tokio::test]
    async fn a_bake_round_trips_through_lance() {
        let dir = tempfile::tempdir().expect("tempdir");
        let conn = connect(dir.path().to_str().expect("utf8"))
            .await
            .expect("connect");

        assert_eq!(
            table_rows(&conn, "geo").await.expect("count"),
            None,
            "a fresh store has no table — distinct from an empty one"
        );

        let bytes = slab_bytes(64);
        assert_eq!(hydrate(&conn, "geo", &bytes).await.expect("hydrate"), 64);
        assert_eq!(table_rows(&conn, "geo").await.expect("count"), Some(64));

        let batch = read_batch(&conn, "geo").await.expect("read").expect("some");
        let col = batch
            .column_by_name(ROW_COLUMN)
            .unwrap()
            .as_any()
            .downcast_ref::<FixedSizeBinaryArray>()
            .unwrap();
        assert_eq!(col.len(), 64);
        assert_eq!(
            col.value_data(),
            &bytes[..],
            "the crystal must survive verbatim"
        );
    }

    /// **Lance's read path gives NO alignment guarantee — and the slab is
    /// borrowed over it anyway.** This is the test the whole module turns on.
    ///
    /// The discovery, in order, because the sequence is the lesson:
    ///
    /// 1. A round trip through a real dataset was refused with
    ///    `Misaligned { align: 64 }`. Measured: `ptr % 8 == 0`, `ptr % 64 == 48`.
    /// 2. The obvious repair — copy into aligned storage — was written, and it
    ///    was **wrong**. Zero copy is a law without escape hatches: *"no cost
    ///    argument can ever favour a copy"*. A guard that leaves the caller
    ///    exactly one lawful option, and that option is a copy, is not a safe
    ///    default; it is the violation wearing a guard's clothes.
    /// 3. The check was in the wrong place. `RowSlab::morton_at` reads BYTES,
    ///    so `lower_bound` and `tile_range` never needed the `&[NodeRow]` cast.
    ///    Alignment moved off construction and onto `rows()`, which is the only
    ///    operation that actually casts.
    ///
    /// A first version of this test asserted `ptr % 64 != 0`; it passed alone
    /// and FAILED in the full suite, because run order changes allocator state.
    /// **"Usually aligned" is the dangerous shape** — it passes CI and reaches
    /// production, where the same cast is fine on one machine and UB on
    /// another. So nothing here asserts an alignment VALUE; it asserts that the
    /// lookup works regardless of one.
    #[tokio::test]
    async fn the_lance_read_path_is_borrowed_at_any_alignment() {
        let dir = tempfile::tempdir().expect("tempdir");
        let conn = connect(dir.path().to_str().expect("utf8"))
            .await
            .expect("connect");
        let bytes = slab_bytes(64);
        hydrate(&conn, "geo", &bytes).await.expect("hydrate");

        let batch = read_batch(&conn, "geo").await.expect("read").expect("some");
        let col = batch
            .column_by_name(ROW_COLUMN)
            .unwrap()
            .as_any()
            .downcast_ref::<FixedSizeBinaryArray>()
            .unwrap();
        let arrow_ptr = col.value_data().as_ptr();

        // THE CLAIM: a Lance read-back slabs, whatever its alignment.
        let slab = slab_of(&batch).expect("a Lance read-back must borrow, never copy");
        assert_eq!(slab.len(), 64);

        // And it BORROWS — pointer identity, because a content check would pass
        // for a copy too.
        assert_eq!(
            slab.as_bytes().as_ptr(),
            arrow_ptr,
            "the slab must point into Lance's own buffer"
        );

        // Every lookup operation works over that borrow, at any alignment,
        // because they are byte arithmetic rather than a cast.
        for i in 0..64 {
            assert_eq!(
                slab.morton_at(i),
                RowSlab::new(&bytes).unwrap().morton_at(i),
                "row {i}'s Morton code must survive the round trip"
            );
        }

        // The `&[NodeRow]` projection is the ONE thing alignment still gates —
        // and its unavailability costs the lookup nothing. Asserted as the
        // biconditional rather than as an alignment VALUE, because the value is
        // nondeterministic (see this module's doc).
        assert_eq!(
            slab.rows().is_some(),
            arrow_ptr as usize % 64 == 0,
            "rows() must be available exactly when Lance's buffer is aligned"
        );
        if let Some(rows) = slab.rows() {
            assert_eq!(rows[0].key.as_bytes()[0], 1);
        }
    }

    /// A truncated artifact is REFUSED, not stored short. A partial hydrate is
    /// the failure mode that looks healthy: the table exists, the count is
    /// plausible, and every lookup past the truncation answers "no".
    #[tokio::test]
    async fn a_truncated_bake_is_refused() {
        let dir = tempfile::tempdir().expect("tempdir");
        let conn = connect(dir.path().to_str().expect("utf8"))
            .await
            .expect("connect");

        let mut bytes = slab_bytes(2);
        bytes.truncate(NODE_ROW_STRIDE + 100);
        assert!(
            hydrate(&conn, "geo", &bytes).await.is_err(),
            "512+100 bytes"
        );
        assert!(hydrate(&conn, "geo", &[]).await.is_err(), "empty");
        // The silence half: the same call with a whole number of rows succeeds,
        // so the refusal is about truncation and not about hydrate being broken.
        assert!(hydrate(&conn, "geo", &slab_bytes(2)).await.is_ok());
    }

    /// **What alignment still gates, now that it no longer gates the slab.**
    ///
    /// Which buffer you get depends on how the array was built, and the
    /// difference is invisible at the type level:
    ///
    /// - `FixedSizeBinaryArray::try_from_iter` (what [`hydrate`] uses)
    ///   allocates through **Arrow's own allocator** — 64-byte aligned.
    /// - `Buffer::from_vec` adopts a `Vec<u8>`'s allocation — **1-byte
    ///   aligned**.
    ///
    /// An earlier version had [`slab_of`] REFUSE the second case. That was the
    /// law violation: it left the caller one lawful option, and that option was
    /// a copy. Now the slab is built either way and every lookup works; only
    /// `RowSlab::rows()` — the one operation that casts — declines.
    ///
    /// Two-sided on purpose: the unaligned buffer must still SLAB and still
    /// answer `morton_at`, and must still decline `rows()`. A version that
    /// silently cast anyway would pass the first half and be UB.
    #[test]
    fn an_unaligned_buffer_still_slabs_and_only_the_cast_declines() {
        use arrow::array::ArrayData;
        let bytes = slab_bytes(6);
        let data = ArrayData::builder(DataType::FixedSizeBinary(NODE_ROW_STRIDE as i32))
            .len(3)
            .offset(2)
            .add_buffer(arrow::buffer::Buffer::from_vec(bytes))
            .build()
            .expect("array data over a Vec-backed buffer");
        let arr = FixedSizeBinaryArray::from(data);

        // Arrow normalised the offset away and rebased the data — the same
        // finding as the slice test, reached from a second construction path.
        assert_eq!(arr.offset(), 0, "arrow normalises the offset away");
        assert_eq!(arr.value(0)[0], 3, "…and rebases to the third row");
        let ptr = arr.value_data().as_ptr() as usize;

        let batch = RecordBatch::try_new(row_schema(), vec![Arc::new(arr)]).expect("batch");
        // Whatever the address, the slab is built and the LOOKUP works. This is
        // the unconditional half.
        let slab = slab_of(&batch).expect("any buffer must slab");
        assert_eq!(slab.len(), 3);
        let _ = slab.morton_at(0);
        assert_eq!(
            slab.as_bytes().as_ptr() as usize,
            ptr,
            "and it borrows, whatever the alignment"
        );

        // The INVARIANT, not a value: the cast is available exactly when the
        // buffer is aligned. An earlier version asserted `ptr % 64 != 0` on the
        // strength of one observation; a `Vec` allocation is sometimes 64-aligned
        // by luck, so that test failed roughly one run in five. Asserting the
        // biconditional is both non-flaky and the thing actually promised.
        assert_eq!(
            slab.rows().is_some(),
            ptr % 64 == 0,
            "rows() must be available exactly when the buffer is 64-aligned"
        );
    }

    /// A wrong-width column is refused. A `FixedSizeBinary(256)` table would
    /// otherwise be slabbed at stride 512 and read every other row as garbage.
    #[test]
    fn a_wrong_stride_column_is_refused() {
        let a = FixedSizeBinaryArray::try_from_iter([[0u8; 256], [1u8; 256]].iter().map(|r| *r))
            .expect("array");
        let schema = Arc::new(Schema::new(vec![Field::new(
            ROW_COLUMN,
            DataType::FixedSizeBinary(256),
            false,
        )]));
        let batch = RecordBatch::try_new(schema, vec![Arc::new(a)]).expect("batch");
        assert!(matches!(slab_of(&batch), Err(AbiError::WrongType { .. })));
    }

    /// A batch without the row column is refused by name, not by position — a
    /// positional read would happily slab whatever column 0 happened to be.
    #[test]
    fn a_batch_without_the_row_column_is_refused() {
        let a = arrow::array::Int64Array::from(vec![1i64, 2, 3]);
        let schema = Arc::new(Schema::new(vec![Field::new(
            "not_the_row_column",
            DataType::Int64,
            false,
        )]));
        let batch = RecordBatch::try_new(schema, vec![Arc::new(a)]).expect("batch");
        assert_eq!(slab_of(&batch).unwrap_err(), AbiError::NoRowColumn);
    }
}
