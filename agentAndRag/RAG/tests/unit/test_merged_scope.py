import json
import numpy as np
import pytest

from RAG.simple_rag.category_index import merged_search_scope
from RAG.simple_rag.vector_store import NumpyVectorStore, StoreConfig


def test_union_scope_never_leaks_and_mmap_matches_dense(tmp_path):
    store = NumpyVectorStore(tmp_path)
    store.init_new(StoreConfig(dim=2))
    vectors = np.array([[1, 0], [.6, .8], [0, 1], [.8, .6]], dtype=np.float32)
    store.add(vectors, [{"chunk_id": str(i), "text": str(i)} for i in range(4)])
    (tmp_path / 'category_rows.json').write_text(json.dumps({'a': [0, 1], 'b': [1, 2], 'c': [3]}))
    store.load()
    assert isinstance(store._emb, np.memmap)
    assert [m['chunk_id'] for m, _ in store.search(vectors[0], 10, categories=('b',))] == ['1', '2']
    assert {m['chunk_id'] for m, _ in store.search(vectors[0], 10, categories=('a', 'b'))} == {'0', '1', '2'}
    assert store.search(vectors[0], 5, categories=('unknown',)) == []
    assert store.search(vectors[0], 0) == []
    with pytest.raises(ValueError):
        store.search(np.array([np.nan, 0]), 1)


def test_taxonomy_atomic_replacement_changes_release_without_cross_category_fallback(tmp_path):
    path = tmp_path / 'RAG/data/category_taxonomy.json'
    path.parent.mkdir(parents=True)
    data = {'merged_index': 'releases/one', 'categories': [{'id': 'a.1', 'chunk_count': 2}, {'id': 'b.1', 'chunk_count': 3}]}
    path.write_text(json.dumps(data))
    assert merged_search_scope(tmp_path, 'a.*') == (tmp_path / 'releases/one', ('a.1',))
    assert merged_search_scope(tmp_path, 'missing')[1] == ()
    data['merged_index'] = 'releases/two-new'
    replacement = path.with_suffix('.tmp')
    replacement.write_text(json.dumps(data)); replacement.replace(path)
    assert merged_search_scope(tmp_path, None) == (tmp_path / 'releases/two-new', ('a.1', 'b.1'))
