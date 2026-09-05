import asyncio
import json

from agent_api.app.observability import jsonl_trace


def test_trace_is_metadata_only_and_drains_on_shutdown(monkeypatch):
    async def scenario():
        records = []
        monkeypatch.setenv('AGENT_TRACE_SAMPLE_RATE', '1')
        monkeypatch.setattr(jsonl_trace, '_append', lambda batch: records.extend(batch))
        jsonl_trace.write_trace('../private/path', tool='secret-tool-name',
                               request={'query': 'private patient', 'messages': [{'content': 'secret'}], 'Authorization': 'credential'},
                               response={'answer': 'private answer', 'tools_called': ['secret-name']}, error='private error')
        await jsonl_trace.close_trace_writer()
        assert len(records) == 1
        serialized = json.dumps(records)
        for secret in ('private', 'secret', 'credential', 'Authorization'):
            assert secret not in serialized
        assert records[0]['request']['query_chars'] == len('private patient')
        assert records[0]['failed'] is True
        assert len(records[0]['trace_id']) == 32
    asyncio.run(scenario())
