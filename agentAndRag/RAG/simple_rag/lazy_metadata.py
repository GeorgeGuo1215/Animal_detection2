"""Read immutable JSONL metadata without retaining the full corpus text in RAM."""
from __future__ import annotations

import json
import mmap
import threading
from array import array
from collections import OrderedDict
from collections.abc import Mapping, Sequence
from pathlib import Path


class SourceTexts(Mapping):
    def __init__(self, metadata, rows):
        self.metadata, self.rows = metadata, rows

    def __getitem__(self, index):
        return self.metadata[self.rows[index]].get('text', '')

    def __iter__(self):
        return iter(self.rows)

    def __len__(self):
        return len(self.rows)


class JsonlMetadata(Sequence):
    """Offsets/source positions are resident; parsed text has a bounded LRU."""
    def __init__(self, path: Path, *, cache_size: int = 512):
        self.handle = path.open('rb')
        self.mapping = mmap.mmap(self.handle.fileno(), 0, access=mmap.ACCESS_READ) if path.stat().st_size else None
        self.offsets = array('Q')
        self.ends = array('Q')
        self.chunk_ids = set()
        self.source_rows: dict[str, dict[int, int]] = {}
        self.cache_size = cache_size
        self.cache: OrderedDict[int, dict] = OrderedDict()
        self.lock = threading.Lock()
        position = 0
        for line in self.handle:
            if line.strip():
                row = json.loads(line)
                index = len(self.offsets)
                self.offsets.append(position); self.ends.append(position + len(line))
                chunk_id = row.get('chunk_id')
                if isinstance(chunk_id, str):
                    self.chunk_ids.add(chunk_id)
                source, chunk = row.get('source_path'), row.get('chunk_index')
                if isinstance(source, str) and isinstance(chunk, int):
                    self.source_rows.setdefault(source, {})[chunk] = index
            position += len(line)

    def __len__(self):
        return len(self.offsets)

    def __getitem__(self, index):
        if isinstance(index, slice):
            return [self[i] for i in range(*index.indices(len(self)))]
        index = int(index)
        if index < 0:
            index += len(self)
        if not 0 <= index < len(self):
            raise IndexError(index)
        with self.lock:
            if index in self.cache:
                self.cache.move_to_end(index)
                return self.cache[index]
        row = json.loads(self.mapping[self.offsets[index]:self.ends[index]])
        with self.lock:
            self.cache[index] = row
            if len(self.cache) > self.cache_size:
                self.cache.popitem(last=False)
        return row

    def source_index(self):
        return {source: SourceTexts(self, rows) for source, rows in self.source_rows.items()}

    def close(self):
        if getattr(self, 'mapping', None) is not None:
            self.mapping.close()
            self.mapping = None
        if getattr(self, 'handle', None) is not None:
            self.handle.close()

    def __del__(self):
        self.close()
