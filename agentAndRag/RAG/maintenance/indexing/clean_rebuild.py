"""Rebuild the active corpus into an immutable, audited semantic-v3 release.

Preparation, embedding and publication are separate commands. The original
corpus is read-only. No candidate release is published implicitly.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import inspect
from dataclasses import asdict
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any

import numpy as np

from RAG.simple_rag.structured_text import VERSION, parse_and_clean, tokenize_chunks
from RAG.simple_rag.embeddings import Embedder
from RAG.maintenance.indexing.rebuild_category_indexes import load_catalog, resolve_book_sources
from agent_api.app.hf_local_model import resolve_embedding_model_id

AGENT_ROOT = Path(__file__).resolve().parents[3]


def write_json(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
    os.replace(temporary, path)


def fingerprint(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def assert_unpublished(output: Path) -> None:
    active = AGENT_ROOT / "RAG/data/category_taxonomy.json"
    if active.exists():
        taxonomy = json.loads(active.read_text(encoding="utf-8"))
        merged = taxonomy.get("merged_index")
        if merged and (AGENT_ROOT / merged).resolve() == (output / "merged").resolve():
            raise RuntimeError("active releases are immutable; choose a new output directory")


def reconstruct_source(rows: list[dict]) -> tuple[str, list[dict]]:
    """Invert adjacent exact overlaps, preserving all non-overlapping text.

    Reconstructed page divisions are editorial sequences, not original folios.
    A source map relates each old chunk to its span in the reconstructed file.
    """
    rows = sorted(rows, key=lambda row: row["chunk_index"])
    if [row["chunk_index"] for row in rows] != list(range(len(rows))):
        raise ValueError("legacy reconstruction requires unique, contiguous chunk indices")
    text = ""
    source_map = []
    previous = ""
    for row in rows:
        value = row["text"]
        overlap = 0
        for n in range(min(len(previous), len(value)), 0, -1):
            if previous[-n:] == value[:n]:
                overlap = n
                break
        # A one-character punctuation match is not sufficient evidence of overlap.
        if overlap < 32:
            overlap = 0
        separator = "\n\n" if text and not overlap else ""
        start = len(text) - overlap + len(separator)
        text += separator + value[overlap:]
        source_map.append({"chunk_id": row["chunk_id"], "chunk_index": row["chunk_index"],
                           "start": start, "end": len(text), "overlap_chars": overlap})
        previous = value
    # Add reproducible page sequence markers at paragraph boundaries (~6000 characters).
    pages, pending, length = [], [], 0
    for paragraph in text.split("\n\n"):
        if pending and length + len(paragraph) > 6000:
            pages.append("\n\n".join(pending))
            pending, length = [], 0
        pending.append(paragraph)
        length += len(paragraph) + 2
    if pending:
        pages.append("\n\n".join(pending))
    result = "\n\n<--- Page Split --->\n\n".join(pages)
    return result, source_map


def prepare(args) -> dict:
    from transformers import AutoTokenizer
    output = Path(args.output_root).resolve()
    assert_unpublished(output)
    output.mkdir(parents=True, exist_ok=True)
    source_index = Path(args.source_index).resolve()
    taxonomy_path = Path(args.taxonomy).resolve()
    taxonomy = json.loads(taxonomy_path.read_text(encoding="utf-8"))
    model = resolve_embedding_model_id(None, AGENT_ROOT)
    tokenizer = AutoTokenizer.from_pretrained(model, local_files_only=True)
    _, catalog = load_catalog(Path(args.classification))
    resolved, _ = resolve_book_sources(source_root=Path(args.source_root), books=catalog)
    sources = {book.book_id: book.source_path for book in resolved}
    original: dict[str, dict] = {}
    reconstruct: dict[str, list[dict]] = defaultdict(list)
    old_counts = Counter()
    for line in (source_index / "meta.jsonl").open(encoding="utf-8"):
        row = json.loads(line)
        book = row["book_id"]
        original.setdefault(book, row)
        old_counts[book] += 1
        if book not in sources:
            reconstruct[book].append(row)
    excluded = {item["book_id"] for item in taxonomy.get("excluded_books", [])}
    if excluded.intersection(original):
        raise ValueError("active corpus unexpectedly contains an excluded book")
    manifest = {
        "version": VERSION, "release_id": output.name,
        "input_taxonomy_sha256": fingerprint(taxonomy_path),
        "input_metadata_sha256": fingerprint(source_index / "meta.jsonl"),
        "embedding_model": model, "chunk_target": 320, "chunk_max": 384, "overlap": 48,
        "books": [], "excluded_books": taxonomy.get("excluded_books", []),
    }
    pipeline_hash = hashlib.sha256((Path(__file__).read_bytes() +
        (AGENT_ROOT / "RAG/simple_rag/structured_text.py").read_bytes())).hexdigest()
    manifest["pipeline_sha256"] = pipeline_hash
    write_json(output / "input_taxonomy.json", taxonomy)
    for number, (book, old) in enumerate(original.items(), 1):
        raw_path = output / "sources" / f"{book}.mmd"
        raw_path.parent.mkdir(parents=True, exist_ok=True)
        reconstruction = None
        if book in sources:
            raw = sources[book].read_bytes()
            if hashlib.sha1(raw).hexdigest() != old["source_sha1"]:
                raise ValueError(f"source fingerprint changed for book {book}; refusing ambiguous source")
            raw_path.write_bytes(raw)
        else:
            raw_text, mapping = reconstruct_source(reconstruct[book])
            raw_path.write_text(raw_text, encoding="utf-8")
            reconstruction = {"method": "exact_adjacent_overlap", "old_source_sha1": old["source_sha1"],
                              "page_sequence_kind": "reconstructed", "offset_basis": "before_page_markers", "chunks": mapping}
            write_json(output / "sources" / f"{book}.reconstruction.json", reconstruction)
        raw_text = raw_path.read_text(encoding="utf-8", errors="replace")
        source_version = fingerprint(raw_path)
        chunks_path = output / "books" / book / "meta.jsonl"
        stats_path = chunks_path.parent / "audit.json"
        if stats_path.exists() and chunks_path.exists():
            cached = json.loads(stats_path.read_text(encoding="utf-8"))
            if cached.get("source_version") == source_version and cached.get("pipeline_sha256") == pipeline_hash:
                manifest["books"].append(cached)
                print(f"[{number}/{len(original)}] {book}: reused prepared chunks", flush=True)
                continue
        blocks, quarantine = parse_and_clean(raw_text)
        structure_hash = hashlib.sha256(json.dumps([asdict(b) for b in blocks], ensure_ascii=False, sort_keys=True).encode()).hexdigest()
        tokenization_hash = hashlib.sha256((inspect.getsource(tokenize_chunks) + model + ':320:384:48').encode()).hexdigest()
        clean_path = output / "cleaned" / f"{book}.mmd"
        cleaned_text = "\n\n".join(b.text for b in blocks)
        if stats_path.exists() and chunks_path.exists() and clean_path.exists():
            cached = json.loads(stats_path.read_text(encoding="utf-8"))
            # A cleaning-rule revision often affects only a few books. Reuse tokenization
            # only when the complete retained stream and its source fingerprint agree.
            if (cached.get("source_version") == source_version
                    and cached.get("structure_sha256") == structure_hash
                    and cached.get("tokenization_sha256") == tokenization_hash
                    and clean_path.read_text(encoding="utf-8") == cleaned_text
                    and cached.get("metadata_sha256") == fingerprint(chunks_path)):
                cached.update(pipeline_sha256=pipeline_hash,
                              quarantine_chars=sum(len(b["text"]) for b in quarantine),
                              quarantine_reasons=dict(Counter(b["reason"] for b in quarantine)))
                with (chunks_path.parent / "quarantine.jsonl").open("w", encoding="utf-8") as handle:
                    for item in quarantine:
                        handle.write(json.dumps({"book_id": book, "source_version": source_version, **item}, ensure_ascii=False) + "\n")
                write_json(stats_path, cached)
                manifest["books"].append(cached)
                print(f"[{number}/{len(original)}] {book}: verified unchanged retained stream", flush=True)
                continue
        chunks = tokenize_chunks(blocks, tokenizer)
        if not chunks:
            raise ValueError(f"cleaning removed all evidence from {book}")
        chunks_path.parent.mkdir(parents=True, exist_ok=True)
        # Keep metadata matching the last vector receipt for exact-text reuse.
        if chunks_path.exists() and (chunks_path.parent / "embedding.json").exists():
            receipt = json.loads((chunks_path.parent / "embedding.json").read_text(encoding="utf-8"))
            if receipt.get("metadata_sha256") == fingerprint(chunks_path):
                shutil.copyfile(chunks_path, chunks_path.parent / "meta.previous.jsonl")
        identities: dict[str, int] = {}
        unique = []
        source_sha1 = hashlib.sha1(raw_path.read_bytes()).hexdigest()
        for chunk in chunks:
            key = chunk["content_hash"]
            if key in identities:
                unique[identities[key]]["duplicate_source_spans"].extend(chunk["source_spans"])
                continue
            identities[key] = len(unique)
            chunk.update({
                "chunk_id": hashlib.sha256(f"{VERSION}:{book}:{source_version}:{len(unique)}:{key}".encode()).hexdigest(),
                "chunk_index": len(unique), "book_id": book, "book_title": old["book_title"],
                "category_ids": old["category_ids"], "source_path": f"books/{book}.mmd", "source_file": f"{book}.mmd",
                "source_sha1": source_sha1,
                "source_version": source_version, "page_sequence_kind": "reconstructed" if reconstruction else "ocr_page_sequence",
                "duplicate_source_spans": [], "quality_status": "retained", "n_units": chunk["token_count"],
            })
            unique.append(chunk)
        with chunks_path.open("w", encoding="utf-8") as handle:
            for chunk in unique:
                handle.write(json.dumps(chunk, ensure_ascii=False) + "\n")
        with (chunks_path.parent / "quarantine.jsonl").open("w", encoding="utf-8") as handle:
            for item in quarantine:
                handle.write(json.dumps({"book_id": book, "source_version": source_version, **item}, ensure_ascii=False) + "\n")
        clean_path = output / "cleaned" / f"{book}.mmd"
        clean_path.parent.mkdir(parents=True, exist_ok=True)
        clean_path.write_text("\n\n".join(b.text for b in blocks), encoding="utf-8")
        stats = {
            "book_id": book, "title": old["book_title"], "source_version": source_version, "pipeline_version": VERSION,
            "pipeline_sha256": pipeline_hash,
            "structure_sha256": structure_hash, "tokenization_sha256": tokenization_hash,
            "page_sequence_kind": "reconstructed" if reconstruction else "ocr_page_sequence",
            "old_chunks": old_counts[book], "chunks": len(unique), "duplicate_chunks": len(chunks)-len(unique),
            "raw_chars": len(raw_text), "retained_chars": sum(len(b.text) for b in blocks),
            "quarantine_chars": sum(len(b["text"]) for b in quarantine),
            "quarantine_reasons": dict(Counter(b["reason"] for b in quarantine)),
            "retained_tables": sum(b.kind == "table" for b in blocks),
            "max_tokens": max(c["token_count"] for c in unique),
            "metadata_sha256": fingerprint(chunks_path),
        }
        write_json(stats_path, stats)
        manifest["books"].append(stats)
        print(f"[{number}/{len(original)}] {book}: {old_counts[book]} -> {len(unique)} chunks; quarantine={len(quarantine)}", flush=True)
    write_json(output / "manifest.json", manifest)
    return manifest


def embed(args) -> dict:
    output = Path(args.output_root).resolve()
    assert_unpublished(output)
    manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
    model = resolve_embedding_model_id(None, AGENT_ROOT)
    embedder = Embedder(model, device=args.device)
    model_limit = int(embedder.model.max_seq_length)
    for book in manifest["books"]:
        folder = output / "books" / book["book_id"]
        vectors_path = folder / "embeddings.npy"
        receipt_path = folder / "embedding.json"
        if vectors_path.exists() and receipt_path.exists():
            receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
            if (receipt.get("metadata_sha256") == book["metadata_sha256"] and receipt.get("model") == model
                    and receipt.get("vectors_sha256") == fingerprint(vectors_path)):
                print(f"{book['book_id']}: reused vectors", flush=True)
                continue
        metas = [json.loads(line) for line in (folder / "meta.jsonl").open(encoding="utf-8")]
        texts = [m["text"] for m in metas]
        lengths = embedder.model.tokenizer(["passage: " + t for t in texts], truncation=False, add_special_tokens=True, return_length=True)["length"]
        if max(lengths) > model_limit:
            raise ValueError(f"book {book['book_id']} exceeds embedding input limit")
        previous = folder / "meta.previous.jsonl"
        old_vectors = None
        old_rows = {}
        if vectors_path.exists() and receipt_path.exists() and previous.exists():
            receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
            if (receipt.get("metadata_sha256") == fingerprint(previous) and receipt.get("model") == model
                    and receipt.get("vectors_sha256") == fingerprint(vectors_path)):
                old_vectors = np.load(vectors_path, mmap_mode="r", allow_pickle=False)
                old_rows = {json.loads(line)["text"]: i for i, line in enumerate(previous.open(encoding="utf-8"))}
        missing = [i for i, text in enumerate(texts) if text not in old_rows]
        if old_vectors is not None and all(i < len(old_vectors) for i in old_rows.values()):
            vectors = np.empty((len(texts), old_vectors.shape[1]), dtype=np.float32)
            for i, text in enumerate(texts):
                if text in old_rows:
                    vectors[i] = old_vectors[old_rows[text]]
            if missing:
                vectors[missing] = embedder.embed_texts([texts[i] for i in missing], batch_size=args.batch_size).vectors
        else:
            vectors = embedder.embed_texts(texts, batch_size=args.batch_size).vectors
            missing = list(range(len(texts)))
        del old_vectors
        temporary = folder / "embeddings.build.npy"
        np.save(temporary, vectors)
        os.replace(temporary, vectors_path)
        write_json(folder / "store_config.json", {"dim": int(vectors.shape[1]), "metric": "cosine_dot"})
        write_json(receipt_path, {"model": model, "metadata_sha256": book["metadata_sha256"], "vectors_sha256": fingerprint(vectors_path)})
        print(f"{book['book_id']}: embedded {len(missing)}, reused {len(texts)-len(missing)} chunks", flush=True)
    return assemble(output)


def assemble(output: Path) -> dict:
    manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
    taxonomy = json.loads((output / "input_taxonomy.json").read_text(encoding="utf-8"))
    total = sum(b["chunks"] for b in manifest["books"])
    first = np.load(output / "books" / manifest["books"][0]["book_id"] / "embeddings.npy", mmap_mode="r")
    dim = first.shape[1]
    merged = output / "merged"
    merged.mkdir(exist_ok=True)
    matrix = np.lib.format.open_memmap(merged / "embeddings.build.npy", mode="w+", dtype=np.float32, shape=(total, dim))
    categories: dict[str, list[int]] = defaultdict(list)
    index = 0
    with (merged / "meta.jsonl").open("w", encoding="utf-8") as handle:
        for book in manifest["books"]:
            folder = output / "books" / book["book_id"]
            vectors = np.load(folder / "embeddings.npy", mmap_mode="r")
            if vectors.shape != (book["chunks"], dim) or not np.isfinite(vectors).all():
                raise ValueError(f"invalid embeddings for {book['book_id']}")
            matrix[index:index+len(vectors)] = vectors
            for line in (folder / "meta.jsonl").open(encoding="utf-8"):
                meta = json.loads(line)
                for category in meta["category_ids"]:
                    categories[category].append(index)
                handle.write(line)
                index += 1
    if index != total:
        raise ValueError("merged vector/metadata count mismatch")
    matrix.flush()
    del matrix
    os.replace(merged / "embeddings.build.npy", merged / "embeddings.npy")
    write_json(merged / "store_config.json", {"dim": dim, "metric": "cosine_dot"})
    write_json(merged / "category_rows.json", categories)
    taxonomy.update(version=3, release_id=output.name, total_chunks=total,
                    chunking={"version": VERSION, "target_tokens": 320, "max_tokens": 384, "overlap_tokens": 48},
                    source_index=str(merged.relative_to(AGENT_ROOT)).replace("\\", "/"),
                    merged_index=str(merged.relative_to(AGENT_ROOT)).replace("\\", "/"))
    for category in taxonomy["categories"]:
        category["chunk_count"] = len(categories.get(category["id"], []))
        category["index_dir"] = taxonomy["merged_index"]
    write_json(output / "taxonomy.json", taxonomy)
    receipt = {"release_id": output.name, "total_chunks": total, "dim": dim,
               "files": {name: fingerprint(merged / name) for name in ("embeddings.npy", "meta.jsonl", "store_config.json", "category_rows.json")}}
    write_json(output / "validated.json", receipt)
    return receipt


def publish(args) -> None:
    output = Path(args.output_root).resolve()
    if not (output / "acceptance.json").exists():
        raise RuntimeError("quality/performance acceptance.json is required before publication")
    acceptance = json.loads((output / "acceptance.json").read_text(encoding="utf-8"))
    if acceptance.get("passed") is not True:
        raise RuntimeError("release acceptance has not passed")
    if acceptance.get("release_id") != output.name:
        raise RuntimeError("acceptance belongs to another release")
    for name in ("manifest.json", "taxonomy.json", "validated.json"):
        if acceptance.get("artifacts", {}).get(name) != fingerprint(output / name):
            raise RuntimeError(f"acceptance does not bind current artifact: {name}")
    for gate in ("quality", "performance", "reproducibility"):
        if acceptance.get("gates", {}).get(gate, {}).get("passed") is not True:
            raise RuntimeError(f"release gate has not passed: {gate}")
    receipt = json.loads((output / "validated.json").read_text(encoding="utf-8"))
    for name, expected in receipt["files"].items():
        if fingerprint(output / "merged" / name) != expected:
            raise RuntimeError(f"release file changed after validation: {name}")
    target = Path(args.taxonomy).resolve()
    current = json.loads(target.read_text(encoding="utf-8"))
    if current.get("release_id") == output.name:
        return
    manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
    if fingerprint(target) != manifest["input_taxonomy_sha256"]:
        raise RuntimeError("active taxonomy changed since preparation; review before switching")
    write_json(output / "previous_taxonomy.json", current)
    write_json(target, json.loads((output / "taxonomy.json").read_text(encoding="utf-8")))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stage", choices=("prepare", "embed", "publish"))
    parser.add_argument("--source-root", default="")
    parser.add_argument("--source-index", default=str(AGENT_ROOT / "RAG/data/rag_index_e5"))
    parser.add_argument("--taxonomy", default=str(AGENT_ROOT / "RAG/data/category_taxonomy.json"))
    parser.add_argument("--classification", default=str(AGENT_ROOT / "RAG/data/veterinary_materials_classification_2.0.xlsx"))
    parser.add_argument("--output-root", required=True)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--batch-size", type=int, default=32)
    args = parser.parse_args()
    {"prepare": prepare, "embed": embed, "publish": publish}[args.stage](args)


if __name__ == "__main__":
    main()
