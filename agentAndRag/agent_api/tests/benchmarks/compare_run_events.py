"""Real PostgreSQL run/event comparison with a deterministic 100-delta model stub.
Use only a disposable database with no connected worker.
"""
import argparse
import asyncio
import importlib.util
import json
import os
import sys
import time
from pathlib import Path

import numpy as np
import pytest
from sqlalchemy import event

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--database-url', required=True)
parser.add_argument('--baseline-source', required=True, type=Path)
parser.add_argument('--output', required=True, type=Path)
args=parser.parse_args()
args.output.parent.mkdir(parents=True, exist_ok=True)
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT))
os.environ['PETMIND_TEST_POSTGRES_URL']=args.database_url
os.environ['AGENT_PLATFORM_ENV']='development'
from agent_api.app.platform.database import get_platform_engine, platform_session
from agent_api.app.platform.models import UserPreference
from agent_api.app.platform.runs import service
from agent_api.tests.platform.test_run_lifecycle import _setup, _teardown, _FakeOrchestrator, _install_fake_agent

spec=importlib.util.spec_from_file_location('agent_api.app.platform.runs._baseline_perf',args.baseline_source)
baseline=importlib.util.module_from_spec(spec);sys.modules[spec.name]=baseline;spec.loader.exec_module(baseline)

async def main():
    monkeypatch=pytest.MonkeyPatch()
    _install_fake_agent(monkeypatch,_FakeOrchestrator([{'status':'streaming','content':'word '}]*100))
    for name in ['load_user_memory','write_user_memory','build_moe_orchestrator','public_moe_allowed_tools']:
        setattr(baseline,name,getattr(service,name))
    result={}
    for name,module in [('baseline',baseline),('candidate',service)]:
        batches=[]
        for repeat in range(3):
            for concurrency in [1,4,8]:
                runs=[]
                for index in range(concurrency):
                    user,run=await _setup(args.output.parent,monkeypatch,f'perf-{name}-{repeat}-{index}')
                    async with platform_session() as session:
                        session.add(UserPreference(user_id=user,memory_recall_enabled=False,memory_write_enabled=False))
                        await session.commit()
                    runs.append(run)
                engine=get_platform_engine().sync_engine
                commits=0
                def committed(connection):
                    nonlocal commits
                    commits+=1
                event.listen(engine,'commit',committed)
                async def execute(run):
                    started=time.perf_counter()
                    await module.execute_run(run)
                    return (time.perf_counter()-started)*1000
                started=time.perf_counter()
                times=await asyncio.gather(*(execute(run) for run in runs))
                elapsed=time.perf_counter()-started
                event.remove(engine,'commit',committed)
                batches.append({'round':repeat+1,'concurrency':concurrency,'p95_ms':float(np.percentile(times,95)),
                                'throughput_runs_s':concurrency/elapsed,'write_commits_per_run':commits/concurrency})
                print(name,'round',repeat+1,'concurrency',concurrency,'commits/run',commits/concurrency,flush=True)
        _,idle=await _setup(args.output.parent,monkeypatch,'idle-'+name)
        engine=get_platform_engine().sync_engine
        reads=0
        def read(connection,cursor,statement,parameters,context,executemany):
            nonlocal reads
            if statement.lstrip().upper().startswith('SELECT') and ('platform_agent_runs' in statement or 'platform_run_events' in statement): reads+=1
        event.listen(engine,'before_cursor_execute',read)
        try:
            async with asyncio.timeout(11):
                async for _ in module.run_event_stream(idle): pass
        except TimeoutError: pass
        event.remove(engine,'before_cursor_execute',read)
        result[name]={'batches':batches,'idle_duration_s':11,'idle_selects':reads}
        await _teardown()
        args.output.write_text(json.dumps(result,indent=2))
    monkeypatch.undo()

asyncio.run(main())
