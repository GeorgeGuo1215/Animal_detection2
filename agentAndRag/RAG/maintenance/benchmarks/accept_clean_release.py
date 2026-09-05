"""Evaluate quality, performance and provenance before an atomic release switch.

Lexical evidence anchors are regression checks, not clinical validation. The
report deliberately omits retrieved passages and user/model credentials.
"""
from __future__ import annotations

import argparse
import json
import re
import statistics
from pathlib import Path

from RAG.maintenance.benchmarks.compare_clean_release import noise
from RAG.maintenance.indexing.clean_rebuild import fingerprint, write_json


def quality(report, cases):
    rows = {row['id']: row for row in report['rows']}
    if set(rows) != set(cases):
        raise ValueError('benchmark case set does not match the regression fixture')
    hits = evidence = noisy = 0
    for key, case in cases.items():
        row = rows[key]
        if row['query'] != case['query'] or row.get('category') != case.get('category'):
            raise ValueError('benchmark query or scope changed')
        matches = row['hits']
        anchors = case['evidence_patterns']
        evidence += any(not noise(hit['text']) and all(re.search(p, hit['text'], re.I) for p in anchors) for hit in matches)
        noisy += sum(noise(hit['text']) for hit in matches)
        hits += len(matches)
    return {'cases': len(cases), 'evidence_hits': evidence, 'evidence_recall_at_5': evidence / len(cases),
            'noise_hits': noisy, 'hits': hits, 'noise_rate': noisy / max(hits, 1)}


def accept(release, baseline, candidate, audit, cases_path):
    artifacts = {name: fingerprint(release / name) for name in ('manifest.json', 'taxonomy.json', 'validated.json')}
    for report in (candidate, audit):
        if report.get('artifacts') != artifacts:
            raise ValueError('report is not bound to the current release artifacts')
    if candidate.get('cases_sha256') != fingerprint(cases_path):
        raise ValueError('candidate used a different regression fixture')
    cases = {row['id']: row for row in json.loads(cases_path.read_text(encoding='utf-8'))}
    old, new = quality(baseline, cases), quality(candidate, cases)
    comparisons = []
    for concurrency in (1, 4, 8):
        samples = [[row for row in report['performance'] if row['concurrency'] == concurrency] for report in (baseline, candidate)]
        if any(len(rows) < 3 or len({r['round'] for r in rows}) < 3 for rows in samples):
            raise ValueError('three independent rounds required for concurrency 1/4/8')
        before, after = [{key: statistics.median(r[key] for r in rows) for key in ('p95_ms', 'qps', 'rss_mb')} for rows in samples]
        passed = after['p95_ms'] <= before['p95_ms'] * 1.05 and after['qps'] >= before['qps'] * .95 and after['rss_mb'] <= before['rss_mb'] * 1.05
        comparisons.append({'concurrency': concurrency, 'baseline_median': before, 'candidate_median': after, 'passed': passed})
    gates = {
        'quality': {'passed': new['evidence_recall_at_5'] >= old['evidence_recall_at_5'] and new['noise_rate'] < old['noise_rate'], 'baseline': old, 'candidate': new},
        'performance': {'passed': all(row['passed'] for row in comparisons), 'max_regression_fraction': .05, 'comparisons': comparisons},
        'reproducibility': {'passed': audit.get('passed') is True and audit.get('release_id') == release.name, 'totals': audit['totals']},
    }
    return {'release_id': release.name, 'passed': all(g['passed'] for g in gates.values()), 'artifacts': artifacts,
            'cases_sha256': fingerprint(cases_path), 'gates': gates,
            'limitations': ['Fixed 24-query lexical regression set is not a clinical gold standard.',
                            'Residual OCR/navigation noise remains; quarantine is reversible.',
                            'GPU thermal throttling affects absolute timings; report three-round medians and cold start separately.']}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('release', 'baseline', 'candidate', 'audit', 'cases'):
        parser.add_argument('--' + name, required=True, type=Path)
    args = parser.parse_args()
    reports = [json.loads(path.read_text(encoding='utf-8')) for path in (args.baseline, args.candidate, args.audit)]
    result = accept(args.release, *reports, args.cases)
    result['report_sha256'] = {name: fingerprint(getattr(args, name)) for name in ('baseline', 'candidate', 'audit')}
    write_json(args.release / 'acceptance.json', result)
    print(json.dumps({'passed': result['passed'], 'gates': result['gates']}, ensure_ascii=False))
    raise SystemExit(0 if result['passed'] else 1)


if __name__ == '__main__':
    main()
