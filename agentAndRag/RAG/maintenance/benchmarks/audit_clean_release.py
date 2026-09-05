"""Verify an immutable release's provenance and deterministic source reconstruction.

The report contains counts and hashes, never book text. This verifies OCR fidelity,
not the medical accuracy or currency of the source publication.
"""
from __future__ import annotations

import argparse
import json
from collections import Counter
from pathlib import Path

from RAG.maintenance.indexing.clean_rebuild import fingerprint, reconstruct_source, write_json
from RAG.simple_rag.structured_text import content_hash, parse_and_clean


def audit(release: Path, legacy: Path):
    manifest = json.loads((release / 'manifest.json').read_text(encoding='utf-8'))
    totals = Counter()
    reasons = Counter()
    errors = []
    reconstructed = {b['book_id'] for b in manifest['books'] if b['page_sequence_kind'] == 'reconstructed'}
    old = {book: [] for book in reconstructed}
    for line in legacy.open(encoding='utf-8'):
        row = json.loads(line)
        if row['book_id'] in old:
            old[row['book_id']].append(row)
    for book in manifest['books']:
        key = book['book_id']
        source = release / 'sources' / f'{key}.mmd'
        raw = source.read_text(encoding='utf-8', errors='replace')
        if fingerprint(source) != book['source_version']:
            errors.append(f'{key}: source hash mismatch')
        blocks, quarantine = parse_and_clean(raw)
        by_span = {(b.start, b.end): b for b in blocks if b.kind != 'table'}
        reasons.update(item['reason'] for item in quarantine)
        totals.update(books=1, raw_chars=len(raw), retained_chars=sum(len(b.text) for b in blocks),
                      quarantine_chars=sum(len(b['text']) for b in quarantine), quarantine_blocks=len(quarantine),
                      tables=sum(b.kind == 'table' for b in blocks))
        metadata = release / 'books' / key / 'meta.jsonl'
        if fingerprint(metadata) != book['metadata_sha256']:
            errors.append(f'{key}: metadata hash mismatch')
        unique = set()
        for index, line in enumerate(metadata.open(encoding='utf-8')):
            row = json.loads(line)
            totals['chunks'] += 1
            if row['chunk_index'] != index or row['content_hash'] != content_hash(row['text']):
                errors.append(f'{key}:{index}: identity mismatch')
            if row['content_hash'] in unique:
                errors.append(f'{key}:{index}: duplicate retained content')
            unique.add(row['content_hash'])
            spans = row['source_spans']
            if not spans or any(not 0 <= a < b <= len(raw) for a, b in spans):
                errors.append(f'{key}:{index}: invalid source span')
            if row['source_version'] != book['source_version'] or row['page_sequence_kind'] != book['page_sequence_kind']:
                errors.append(f'{key}:{index}: source version mismatch')
            # Every prose window must be an exact substring of its cleaned source blocks.
            # Only the separately recorded section prefix may be prepended.
            if row['content_type'] != 'table':
                group = [by_span.get(tuple(span)) for span in spans]
                if all(group):
                    text = '\n\n'.join(b.text for b in group)
                    value = text[row['window_start']:row['window_end']]
                    if not value or not row['text'].endswith(value):
                        errors.append(f'{key}:{index}: altered prose window')
                    totals['prose_windows_verified'] += 1
                else:
                    errors.append(f'{key}:{index}: missing retained source block')
            else:
                totals['table_windows'] += 1
            if not 0 < row['token_count'] <= manifest['chunk_max']:
                errors.append(f'{key}:{index}: token budget')
        if key in old:
            text, mapping = reconstruct_source(old[key])
            if text != raw:
                errors.append(f'{key}: reconstruction is not reproducible')
            totals['reconstructed_old_chunks'] += len(mapping)
            totals['reconstructed_exact_overlaps'] += sum(m['overlap_chars'] > 0 for m in mapping)
        print(key, 'verified', flush=True)
    return {'passed': not errors, 'release_id': release.name, 'totals': dict(totals),
            'artifacts': {name: fingerprint(release / name) for name in ('manifest.json', 'taxonomy.json', 'validated.json')},
            'quarantine_reasons': dict(reasons), 'errors': errors,
            'limitations': ['OCR fidelity is not independent clinical validation.',
                            'Tables are covered by parser unit cases and retained raw spans; complete cell accuracy requires source-page review.',
                            'Page sequence is not a printed page number.']}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--release', required=True, type=Path)
    parser.add_argument('--legacy-metadata', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    result = audit(args.release, args.legacy_metadata)
    write_json(args.output, result)
    raise SystemExit(0 if result['passed'] else 1)
