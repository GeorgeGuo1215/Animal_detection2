"""Unit tests for the vitals_alert MCP server (no live database required).

数据库访问通过替换 db._connect 注入假连接，所以这些用例完全离线，
也不会加载任何模型 / 占用显卡。

Run: pytest agent_api/tests/mcp_servers/test_vitals_alert.py -q
"""
import json
import os
import sys
from datetime import datetime, timezone

# repo root (agentAndRag/) so that `mcp_servers.*` is importable
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..")))

import pytest

from mcp_servers.vitals_alert import config as cfg_mod
from mcp_servers.vitals_alert import db as db_mod
from mcp_servers.vitals_alert import server as srv
from mcp_servers.vitals_alert.config import (
    DEFAULT_ABNORMAL_RATIO,
    DEFAULT_DSN,
    DEFAULT_THRESHOLDS,
    HEART_RATE,
    RESPIRATORY_RATE,
    load_config,
    merge_thresholds,
    normalize_species,
)
from mcp_servers.vitals_alert.server import (
    check_vitals,
    classify_alert,
    classify_metric,
    coerce_hours,
)

_VITALS_ENV = (
    "VITALS_DB_DSN",
    "VITALS_METRIC_TABLE",
    "VITALS_PET_TABLE",
    "VITALS_THRESHOLDS_JSON",
    "VITALS_ABNORMAL_RATIO",
    "VITALS_DB_CONNECT_TIMEOUT",
)


@pytest.fixture(autouse=True)
def _clean_env(monkeypatch):
    """Every test starts from an unconfigured environment."""
    for name in _VITALS_ENV:
        monkeypatch.delenv(name, raising=False)


# ---------------------------------------------------------------- fake driver


class FakeCursor:
    """Replays a scripted result per execute() call and records the SQL."""

    def __init__(self, script):
        self.script = list(script)
        self.calls = []  # list of (rendered_sql, params)
        self._current = None

    def execute(self, query, params=None):
        rendered = query.as_string() if hasattr(query, "as_string") else str(query)
        self.calls.append((rendered, params))
        self._current = self.script.pop(0) if self.script else None

    def fetchone(self):
        if isinstance(self._current, list):
            return self._current[0] if self._current else None
        return self._current

    def fetchall(self):
        if isinstance(self._current, list):
            return self._current
        return [] if self._current is None else [self._current]

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False


class FakeConn:
    def __init__(self, script):
        self.cursor_obj = FakeCursor(script)
        self.row_factory = None

    def cursor(self, row_factory=None):
        self.row_factory = row_factory
        return self.cursor_obj

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False


def _install_fake_db(monkeypatch, script):
    conn = FakeConn(script)
    monkeypatch.setattr(db_mod, "_connect", lambda cfg: conn)
    return conn


def _stats_row(**overrides):
    row = {
        "n": 120,
        "hr_n": 120, "hr_avg": 88.5, "hr_min": 62, "hr_max": 145,
        "hr_above": 3, "hr_below": 0,
        "rr_n": 120, "rr_avg": 22.1, "rr_min": 12, "rr_max": 38,
        "rr_above": 1, "rr_below": 0,
        "first_ts": datetime(2026, 7, 30, 18, 0, tzinfo=timezone.utc),
        "last_ts": datetime(2026, 7, 31, 18, 0, tzinfo=timezone.utc),
    }
    row.update(overrides)
    return row


def _empty_stats_row():
    return _stats_row(
        n=0, hr_n=0, hr_avg=None, hr_min=None, hr_max=None, hr_above=0, hr_below=0,
        rr_n=0, rr_avg=None, rr_min=None, rr_max=None, rr_above=0, rr_below=0,
        first_ts=None, last_ts=None,
    )


# ------------------------------------------------------------ config: 阈值


def test_builtin_thresholds_match_upstream_reference():
    # 与 PetHealth_Server 的 pet-health-ranges.ts 对齐
    assert DEFAULT_THRESHOLDS["DOG"][HEART_RATE] == {"min": 60.0, "max": 140.0}
    assert DEFAULT_THRESHOLDS["DOG"][RESPIRATORY_RATE] == {"min": 10.0, "max": 35.0}
    assert DEFAULT_THRESHOLDS["CAT"][HEART_RATE] == {"min": 120.0, "max": 220.0}
    assert DEFAULT_THRESHOLDS["BIRD"][HEART_RATE] == {"min": 200.0, "max": 600.0}
    assert DEFAULT_THRESHOLDS["RABBIT"][RESPIRATORY_RATE] == {"min": 30.0, "max": 60.0}


def test_override_touches_only_the_named_species_and_metric(monkeypatch):
    monkeypatch.setenv(
        "VITALS_THRESHOLDS_JSON",
        json.dumps({"DOG": {"heartRate": {"min": 55, "max": 150}}}),
    )
    cfg = load_config()
    assert cfg.thresholds["DOG"][HEART_RATE] == {"min": 55.0, "max": 150.0}
    # 同物种的另一个指标、以及其他物种都不受影响
    assert cfg.thresholds["DOG"][RESPIRATORY_RATE] == {"min": 10.0, "max": 35.0}
    assert cfg.thresholds["CAT"][HEART_RATE] == {"min": 120.0, "max": 220.0}


def test_override_accepts_upstream_metric_aliases():
    merged = merge_thresholds(
        DEFAULT_THRESHOLDS, {"cat": {"breathRate": {"min": 18, "max": 34}}}
    )
    assert merged["CAT"][RESPIRATORY_RATE] == {"min": 18.0, "max": 34.0}


def test_builtin_table_is_not_mutated_by_a_merge():
    merge_thresholds(DEFAULT_THRESHOLDS, {"DOG": {"heart_rate": {"min": 1, "max": 2}}})
    assert DEFAULT_THRESHOLDS["DOG"][HEART_RATE] == {"min": 60.0, "max": 140.0}


def test_invalid_threshold_json_falls_back_to_defaults(monkeypatch):
    monkeypatch.setenv("VITALS_THRESHOLDS_JSON", "{not json")
    cfg = load_config()
    assert cfg.thresholds["DOG"][HEART_RATE] == {"min": 60.0, "max": 140.0}


@pytest.mark.parametrize(
    "override",
    [
        {"DRAGON": {"heart_rate": {"min": 1, "max": 2}}},   # 未知物种
        {"DOG": {"bloodPressure": {"min": 1, "max": 2}}},   # 未知指标
        {"DOG": {"heart_rate": {"min": 200, "max": 100}}},  # min >= max
        {"DOG": {"heart_rate": {"min": "low", "max": 100}}},  # 非数值
        {"DOG": "nonsense"},                                 # 结构不对
        "nonsense",                                          # 顶层不是对象
    ],
)
def test_bad_override_entries_are_dropped_without_losing_defaults(override):
    merged = merge_thresholds(DEFAULT_THRESHOLDS, override)
    assert merged["DOG"][HEART_RATE] == {"min": 60.0, "max": 140.0}
    assert merged["CAT"][HEART_RATE] == {"min": 120.0, "max": 220.0}


def test_partially_valid_override_keeps_the_good_half():
    merged = merge_thresholds(
        DEFAULT_THRESHOLDS,
        {"DOG": {"heart_rate": {"min": 50, "max": 160}, "respiratory_rate": {"min": 9, "max": 9}}},
    )
    assert merged["DOG"][HEART_RATE] == {"min": 50.0, "max": 160.0}
    assert merged["DOG"][RESPIRATORY_RATE] == {"min": 10.0, "max": 35.0}


# ----------------------------------------------------- config: 环境变量边界


def test_blank_env_is_treated_as_unset(monkeypatch):
    # mcp_servers.json 的 ${VAR} 模板在变量未定义时会展开成空串
    for name in _VITALS_ENV:
        monkeypatch.setenv(name, "")
    cfg = load_config()
    assert cfg.dsn == DEFAULT_DSN
    assert cfg.metric_table == "PetHealthMetric"
    assert cfg.pet_table == "Pet"
    assert cfg.abnormal_ratio == DEFAULT_ABNORMAL_RATIO
    assert cfg.connect_timeout == 5


@pytest.mark.parametrize("raw", ["0", "1.5", "-0.2", "abc"])
def test_out_of_range_abnormal_ratio_falls_back(monkeypatch, raw):
    monkeypatch.setenv("VITALS_ABNORMAL_RATIO", raw)
    assert load_config().abnormal_ratio == DEFAULT_ABNORMAL_RATIO


def test_custom_env_values_are_honoured(monkeypatch):
    monkeypatch.setenv("VITALS_DB_DSN", "postgresql://u:p@db:15432/PetHealth")
    monkeypatch.setenv("VITALS_METRIC_TABLE", "MetricsArchive")
    monkeypatch.setenv("VITALS_PET_TABLE", "Animals")
    monkeypatch.setenv("VITALS_ABNORMAL_RATIO", "0.35")
    monkeypatch.setenv("VITALS_DB_CONNECT_TIMEOUT", "12")
    cfg = load_config()
    assert cfg.dsn == "postgresql://u:p@db:15432/PetHealth"
    assert cfg.metric_table == "MetricsArchive"
    assert cfg.pet_table == "Animals"
    assert cfg.abnormal_ratio == 0.35
    assert cfg.connect_timeout == 12


@pytest.mark.parametrize(
    "raw,expected",
    [("dog", "DOG"), ("Cat", "CAT"), (" bird ", "BIRD"), ("dragon", "OTHER"), (None, "OTHER"), ("", "OTHER")],
)
def test_normalize_species(raw, expected):
    assert normalize_species(raw) == expected


# ------------------------------------------------------------- 告警分级逻辑


def test_metric_is_normal_without_any_breach():
    assert classify_metric(100, 88.0, 0, {"min": 60, "max": 140}, 0.2) == "normal"


def test_metric_is_normal_when_there_are_no_samples():
    assert classify_metric(0, None, 0, {"min": 60, "max": 140}, 0.2) == "normal"


def test_sparse_breach_is_only_a_warning():
    # 10 / 100 = 0.1，低于 0.2 的升级线
    assert classify_metric(100, 88.0, 10, {"min": 60, "max": 140}, 0.2) == "warning"


def test_breach_ratio_at_the_threshold_escalates_to_alert():
    # 20 / 100 = 0.2，正好达到升级线
    assert classify_metric(100, 88.0, 20, {"min": 60, "max": 140}, 0.2) == "alert"


@pytest.mark.parametrize("avg", [59.9, 140.1])
def test_mean_outside_the_range_is_always_an_alert(avg):
    # 均值本身越界说明是持续偏离，不看占比
    assert classify_metric(100, avg, 1, {"min": 60, "max": 140}, 0.9) == "alert"


def test_overall_level_is_the_worst_metric():
    assert classify_alert(["normal", "warning"]) == "warning"
    assert classify_alert(["warning", "alert"]) == "alert"
    assert classify_alert(["normal", "normal"]) == "normal"
    assert classify_alert([]) == "normal"


# ------------------------------------------------------------------ 入参校验


@pytest.mark.parametrize("pet_id", [None, "", "   "])
def test_missing_pet_id_is_rejected_before_touching_the_database(pet_id):
    result = check_vitals(pet_id=pet_id)
    assert result["status"] == "INVALID_ARGUMENT"


@pytest.mark.parametrize(
    "raw,expected",
    [(24, 24), (0, 1), (-5, 1), (9999, 720), ("48", 48), ("abc", 24), (None, 24)],
)
def test_hours_are_clamped_into_the_supported_window(raw, expected):
    assert coerce_hours(raw) == expected


# --------------------------------------------------------------- 数据库分支


def test_happy_path_reports_stats_thresholds_and_alert_level(monkeypatch):
    samples = [
        {"timestamp": datetime(2026, 7, 31, 17, 0, tzinfo=timezone.utc), "hr": 145, "rr": 38},
    ]
    _install_fake_db(monkeypatch, [{"species": "DOG"}, _stats_row(), samples])

    out = check_vitals(pet_id="clx_dog_1", hours=24)

    assert out["status"] == "OK"
    assert out["species"] == "DOG"
    assert out["species_source"] == "database"
    assert out["sample_count"] == 120
    assert out["heart_rate"]["avg"] == 88.5
    assert out["heart_rate"]["max"] == 145
    assert out["respiratory_rate"]["avg"] == 22.1
    assert out["thresholds"][HEART_RATE] == {"min": 60.0, "max": 140.0}
    assert out["abnormal"][HEART_RATE]["count"] == 3
    assert out["abnormal"][HEART_RATE]["above"] == 3
    assert out["abnormal"][HEART_RATE]["ratio"] == 0.025
    # 3/120 与 1/120 都远低于 0.2，且均值在范围内
    assert out["alert_level"] == "warning"
    assert "近 24 小时共 120 条采样" in out["summary"]


def test_evidence_samples_are_split_per_metric(monkeypatch):
    # 一行同时越界，两个指标都该拿到证据；只越一项的行不该串味
    rows = [
        {"timestamp": datetime(2026, 7, 31, 17, 0, tzinfo=timezone.utc), "hr": 145, "rr": 38},
        {"timestamp": datetime(2026, 7, 31, 16, 0, tzinfo=timezone.utc), "hr": 150, "rr": 20},
    ]
    _install_fake_db(monkeypatch, [{"species": "DOG"}, _stats_row(), rows])

    out = check_vitals(pet_id="clx_dog_1")

    hr_samples = out["abnormal"][HEART_RATE]["samples"]
    rr_samples = out["abnormal"][RESPIRATORY_RATE]["samples"]
    assert [s["value"] for s in hr_samples] == [145, 150]
    assert [s["value"] for s in rr_samples] == [38]
    assert hr_samples[0]["timestamp"] == "2026-07-31T17:00:00+00:00"


def test_species_specific_bounds_are_pushed_into_the_sql(monkeypatch):
    conn = _install_fake_db(monkeypatch, [{"species": "CAT"}, _stats_row(), []])

    check_vitals(pet_id="clx_cat_1", hours=6)

    stats_sql, stats_params = conn.cursor_obj.calls[1]
    assert stats_params["hr_min"] == 120.0 and stats_params["hr_max"] == 220.0
    assert stats_params["rr_min"] == 20.0 and stats_params["rr_max"] == 30.0
    assert stats_params["pet_id"] == "clx_cat_1"
    assert stats_params["hours"] == 6
    assert "make_interval(hours => %(hours)s::int)" in stats_sql


def test_explicit_species_skips_the_pet_lookup(monkeypatch):
    conn = _install_fake_db(monkeypatch, [_stats_row(), []])

    out = check_vitals(pet_id="clx_1", species="rabbit")

    assert out["species"] == "RABBIT"
    assert out["species_source"] == "argument"
    # 只有聚合 + 样本两条查询，没有查 Pet 表
    assert len(conn.cursor_obj.calls) == 2
    assert all("FROM \"Pet\"" not in sql for sql, _ in conn.cursor_obj.calls)


def test_unknown_explicit_species_falls_back_to_other_bounds(monkeypatch):
    conn = _install_fake_db(monkeypatch, [_stats_row(), []])

    out = check_vitals(pet_id="clx_1", species="dragon")

    assert out["species"] == "OTHER"
    _, params = conn.cursor_obj.calls[0]
    assert params["hr_min"] == 60.0 and params["hr_max"] == 200.0


def test_window_without_samples_returns_no_data(monkeypatch):
    _install_fake_db(monkeypatch, [{"species": "DOG"}, _empty_stats_row()])

    out = check_vitals(pet_id="clx_dog_1", hours=3)

    assert out["status"] == "NO_DATA"
    assert out["window"]["hours"] == 3


def test_unknown_pet_returns_pet_not_found(monkeypatch):
    _install_fake_db(monkeypatch, [None])

    out = check_vitals(pet_id="ghost")

    assert out["status"] == "PET_NOT_FOUND"
    assert "ghost" in out["message"]


def test_connection_failure_degrades_to_db_unavailable(monkeypatch):
    def _boom(cfg):
        raise OSError("connection refused")

    monkeypatch.setattr(db_mod, "_connect", _boom)

    out = check_vitals(pet_id="clx_1")

    assert out["status"] == "DB_UNAVAILABLE"
    assert "connection refused" in out["message"]


def test_query_failure_also_degrades_to_db_unavailable(monkeypatch):
    class ExplodingCursor(FakeCursor):
        def execute(self, query, params=None):
            raise RuntimeError("relation does not exist")

    conn = FakeConn([])
    conn.cursor_obj = ExplodingCursor([])
    monkeypatch.setattr(db_mod, "_connect", lambda cfg: conn)

    out = check_vitals(pet_id="clx_1")

    assert out["status"] == "DB_UNAVAILABLE"
    assert "relation does not exist" in out["message"]


def test_mean_out_of_range_produces_an_alert(monkeypatch):
    # 猫的心率上限 220，均值 260 属于持续性心动过速
    stats = _stats_row(hr_avg=260.0, hr_above=120, hr_min=240, hr_max=280)
    _install_fake_db(monkeypatch, [{"species": "CAT"}, stats, []])

    out = check_vitals(pet_id="clx_cat_1")

    assert out["alert_level"] == "alert"


# ------------------------------------------------------- 表名注入 / 标识符引用


def test_configured_table_names_are_quoted_identifiers():
    assert 'FROM "PetHealthMetric"' in db_mod.stats_sql("PetHealthMetric").as_string()
    assert 'FROM "Pet"' in db_mod.species_sql("Pet").as_string()


def test_hostile_table_name_cannot_break_out_of_the_identifier():
    rendered = db_mod.stats_sql('x"; DROP TABLE "Pet"; --').as_string()
    # 内部的双引号被转义成 ""，整段仍是一个标识符，不会变成新语句
    assert 'FROM "x""; DROP TABLE ""Pet""; --"' in rendered
    assert "DROP TABLE \"Pet\";" not in rendered.replace('""', '\x00')


def test_custom_table_names_reach_the_generated_sql(monkeypatch):
    monkeypatch.setenv("VITALS_METRIC_TABLE", "MetricsArchive")
    monkeypatch.setenv("VITALS_PET_TABLE", "Animals")
    conn = _install_fake_db(monkeypatch, [{"species": "DOG"}, _stats_row(), []])

    check_vitals(pet_id="clx_1")

    assert 'FROM "Animals"' in conn.cursor_obj.calls[0][0]
    assert 'FROM "MetricsArchive"' in conn.cursor_obj.calls[1][0]


# ------------------------------------------------------------- MCP 契约形状


def test_tool_is_advertised_with_a_usable_schema():
    import asyncio

    tools = asyncio.run(srv.list_tools())
    assert [t.name for t in tools] == ["check_vitals"]
    schema = tools[0].inputSchema
    assert schema["required"] == ["pet_id"]
    assert schema["properties"]["hours"]["default"] == 24
    assert "DOG" in schema["properties"]["species"]["enum"]


def test_call_tool_returns_json_text_content(monkeypatch):
    import asyncio

    _install_fake_db(monkeypatch, [{"species": "DOG"}, _stats_row(), []])
    content = asyncio.run(srv.call_tool("check_vitals", {"pet_id": "clx_1"}))

    payload = json.loads(content[0].text)
    assert payload["status"] == "OK"
    assert payload["pet_id"] == "clx_1"


def test_unknown_tool_name_is_reported_not_raised():
    import asyncio

    content = asyncio.run(srv.call_tool("nope", {}))
    assert "Unknown tool" in json.loads(content[0].text)["error"]


def test_server_process_pulls_in_no_gpu_stack():
    """守住"MCP 子进程不占显卡"这条约束。

    在干净的子进程里导入整个 server 模块，torch / transformers 之类一旦被间接
    引入就会立刻暴露（本进程可能已被别的测试污染，所以必须另起进程判断）。
    """
    import subprocess

    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
    probe = (
        "import sys; import mcp_servers.vitals_alert.server as s; "
        "heavy=[m for m in ('torch','transformers','sentence_transformers') if m in sys.modules]; "
        "print(','.join(heavy))"
    )
    out = subprocess.run(
        [sys.executable, "-c", probe],
        cwd=repo_root,
        capture_output=True,
        text=True,
        timeout=120,
    )
    assert out.returncode == 0, out.stderr
    assert out.stdout.strip() == "", f"unexpected heavy imports: {out.stdout.strip()}"
