from RAG.simple_rag.structured_text import parse_and_clean, content_hash
from RAG.maintenance.indexing.clean_rebuild import reconstruct_source
import pytest


def test_tables_preserve_units_and_headers_and_references_do_not_consume_next_section():
    raw = '''# Treatment
Do not give 2.5 mg/kg if contraindicated.

<table><tr><th>Species</th><th>Dose</th></tr><tr><td>Cat</td><td>2.5 mg/kg</td></tr></table>

# References
Smith 2020 Journal vol. 2

# Monitoring
Check the patient; avoid combining medicines.
'''
    kept, removed = parse_and_clean(raw)
    text = "\n".join(b.text for b in kept)
    assert "Species: Cat | Dose: 2.5 mg/kg" in text
    assert "Do not give 2.5 mg/kg" in text
    assert "avoid combining" in text
    assert "Smith 2020" not in text
    assert any(item["reason"] == "reference_section" for item in removed)


def test_formula_and_numeric_table_are_not_low_density_noise():
    kept, removed = parse_and_clean(r'''# Measurements
\[\sigma = \begin{array}{lll}1 & 2 & 3\end{array}\]

<table><tr><th>mm</th><th>mM</th></tr><tr><td>0.1</td><td>0.01</td></tr></table>''')
    assert any(b.kind == "table" and "0.01" in b.text for b in kept)
    assert not removed
    assert content_hash("1 mM") != content_hash("1 mm")


def test_unmarked_reference_heading_does_not_hide_deeper_chapter():
    kept, removed = parse_and_clean("Further reading\n\nSmith 2020 Journal\n\n## Clinical signs\n\nDo not ignore the following clinical signs.")
    assert any("Do not ignore" in b.text for b in kept)
    assert any("Smith" in b["text"] for b in removed)


def test_markdown_tables_keep_doses_but_quarantine_contents():
    raw = "| Drug | Dose (mg/kg) |\n|---|---|\n| A | 2.5 |\n\n| ORAL CAVITY | 2 |\n|---|---|\n| Diseases | 4 |"
    kept, removed = parse_and_clean(raw)
    assert any("Dose (mg/kg): 2.5" in b.text for b in kept)
    assert any(b["reason"] == "contents_listing" for b in removed)


def test_legacy_reconstruction_preserves_overlap_once_and_rejects_missing_indices():
    overlap = "The animal requires a complete clinical assessment."
    rows = [{"chunk_index": 0, "chunk_id": "a", "text": "# Treatment\n\n" + overlap},
            {"chunk_index": 1, "chunk_id": "b", "text": overlap + "\n\nFollow up."}]
    text, mapping = reconstruct_source(rows)
    assert text.count(overlap) == 1
    assert text.endswith("Follow up.")
    assert mapping[1]["overlap_chars"] == len(overlap)
    with pytest.raises(ValueError):
        reconstruct_source([rows[1]])


def test_unmarked_narrative_after_index_is_recovered_by_common_pipeline():
    paragraph = 'The cell barrier has several distinct components. ' * 6 + 'Do not infer missing doses from damaged OCR.'
    kept, removed = parse_and_clean('Index\n\nAbdomen 12\nBlood 14\n\n' + paragraph)
    assert any(paragraph == block.text for block in kept)
    assert any('Abdomen' in block['text'] for block in removed)

def test_reference_labels_do_not_leak_into_recovered_narrative():
    prose = ('The patient should be monitored and the findings are recorded. '
             'The clinician may review the changes and the animal should have another examination (Study, 2006). ')
    kept, _ = parse_and_clean('# References\n\n' + prose * 2)
    body = [b for b in kept if b.kind == 'text']
    assert body and all('References' not in b.section for b in body)
    kept, _ = parse_and_clean('# References\n\n## Treatment\n\nDo not change 0.01 mg/kg.')
    assert kept[-1].section == 'Treatment'


def test_concatenated_contents_are_quarantined_without_rewriting_doses():
    navigation = 'Section I Cardiopulmonary Disorders ' + ''.join(['AnemiaCoagulopathiesPlateletDysfunctionImmunologicDisorders'] * 8) + ' Section II Hematology'
    kept, removed = parse_and_clean(navigation + '\n\n# Treatment\n\nDo not change 0.01 mg/kg or 1 mM.')
    assert any(b['reason'] == 'concatenated_navigation' for b in removed)
    assert any('Do not change 0.01 mg/kg or 1 mM.' == b.text for b in kept)


def test_navigation_subheadings_and_alphabet_do_not_release_index_entries():
    raw = '## Contents\n\n## Part Two: Systemic Approach\n\nDisease A Disease B\n\n# Treatment\n\nDo not change 0.01 mg/kg.\n\n# Index\n\n## A\n\nAnemia 123\n\n# B\n\nBlood 456'
    kept, removed = parse_and_clean(raw)
    assert not any('Disease A' in b.text or 'Blood 456' in b.text for b in kept)
    assert any('0.01 mg/kg' in b.text for b in kept)
    assert any('Anemia 123' in b['text'] for b in removed)


def test_concatenated_differential_list_is_retained_without_navigation_evidence():
    body = 'HeartwormDiseaseImmuneMediatedHemolyticAnemiaNeoplasiaCardiacDiseasePancreatitisSepsisShock' * 4
    kept, _ = parse_and_clean('# Differential causes\n\n' + body)
    assert any(b.text == body for b in kept)
