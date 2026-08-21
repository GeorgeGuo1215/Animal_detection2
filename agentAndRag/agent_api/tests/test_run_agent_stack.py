from __future__ import annotations

import argparse
import sys

from agent_api.scripts.run_agent_stack import build_child_environment, _commands
from memory_service.scripts.init_local_db import schema_file_names


def test_stack_enables_required_memory_and_keeps_base_environment_isolated():
    """验证进程栈会启用必需的记忆服务，并隔离基础环境。"""
    base = {"OPENAI_MODEL": "test-model"}
    memory_env, agent_env = build_child_environment(
        base,
        memory_host="127.0.0.1",
        memory_port=8300,
        memory_required=True,
    )

    assert base == {"OPENAI_MODEL": "test-model"}
    assert memory_env["MEMORY_PORT"] == "8300"
    assert "AGENT_MEMORY_ENABLED" not in memory_env
    assert agent_env["AGENT_MEMORY_ENABLED"] == "1"
    assert agent_env["AGENT_MEMORY_REQUIRED"] == "1"
    assert agent_env["AGENT_MEMORY_URL"] == "http://127.0.0.1:8300"


def test_stack_commands_use_current_python_and_separate_processes():
    """验证进程栈命令使用当前 Python 且分进程启动。"""
    args = argparse.Namespace(agent_host="0.0.0.0", agent_port=8002)
    memory, agent = _commands(args)

    assert memory == [sys.executable, "-m", "memory_service.app.main"]
    assert agent[:4] == [sys.executable, "-m", "uvicorn", "agent_api.app.main:app"]
    assert agent[-4:] == ["--host", "0.0.0.0", "--port", "8002"]


def test_normal_schema_initialization_does_not_create_pethealth_fixture_tables():
    """验证普通建表不会创建 PetHealth 联调夹具表。"""
    assert schema_file_names(include_fixture=False) == [
        "001_schema.sql",
        "002_memory_subjects_migration.sql",
        "003_memory_derivations.sql",
        "004_remove_turn_provenance.sql",
    ]
    assert schema_file_names(include_fixture=True)[0] == "000_petserver_fixture.sql"
