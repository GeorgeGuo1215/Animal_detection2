"""兽医学生评估对照：对已启动后端跑指令一/二 + 法斗，自动勾选评测清单。

用法：
    # 先启动服务（例：8146）
    python tests/moe/run_moe_vet_eval_live.py --base-url http://127.0.0.1:8146

输出：tests/moe/reports/vet_eval_<timestamp>/
  - case_*.md / case_*_answer.txt
  - checklist.json（供回填 docs/eval_report）
  - SUMMARY.md
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import httpx

_THIS = Path(__file__).resolve()
_AGENT_API = _THIS.parents[2]
_AGENTANDRAG = _THIS.parents[3]
_DOCS = _AGENTANDRAG.parent / "docs"
for _p in (str(_AGENT_API), str(_AGENTANDRAG)):
    if _p not in sys.path:
        sys.path.insert(0, _p)


def _load_dotenv() -> None:
    """从项目 .env 读取环境变量（不覆盖已有值）。"""
    env_path = _AGENTANDRAG / ".env"
    if not env_path.exists():
        return
    try:
        for line in env_path.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            k, v = line.split("=", 1)
            k, v = k.strip(), v.strip().strip('"').strip("'")
            if k and k not in os.environ:
                os.environ[k] = v
    except Exception:  # noqa: BLE001
        pass


_load_dotenv()
os.environ.setdefault("HTTPX_TRUST_ENV", "0")

_GOLDEN = (
    "请帮我看看下面这个病例应该怎么整理和诊断：\n"
    "3 岁多金毛，公的，已经绝育了。昨天晚上开始有点不太对劲，平时特别爱吃，昨天晚饭就没怎么吃。"
    "半夜吐了一次，是黄色的水，今天早上又吐了两次，一次是没消化完的狗粮，一次是黄水。"
    "今天精神比平时差，大部分时间趴着，但是叫它还是会起来。\n"
    "它前天晚上吃过一点鸡骨头，是家里人不小心给的。平时吃商业狗粮，最近没有换粮。"
    "疫苗去年打过，今年还没来得及打，驱虫大概两三个月前做过。没有拉稀，今天早上排便一次，便便成形，没有看到血。"
    "它现在还能喝一点水，但喝完有点想吐。\n"
    "之前身体还可以，没有什么大病，就是小时候肠胃比较敏感。没有咳嗽，也没有流鼻涕。"
    "我们还没带它去医院，也没有给它吃药。"
)

_FLUTD = (
    "5 岁的英短，公猫，已经绝育。今天从早上开始就一直往猫砂盆跑，差不多十几分钟去一次，"
    "每次蹲很久，但是宠主看不出来到底有没有尿出来。猫砂盆里好像只有一点点尿团，比平时少很多。\n"
    "它今天不太爱吃东西，平时早上会主动来要罐头，今天只舔了几口。精神也差一些，老是趴着，还会舔下面。"
    "没有明显呕吐，但是刚才好像干呕了一下。昨天晚上还挺正常的。最近没有换粮，喝水感觉和平时差不多。"
    "宠主家里最近来了客人，它有点紧张，躲了两天。\n"
    "疫苗应该是去年打的，驱虫不太记得。之前有过一次尿血，可能是膀胱炎，吃药后好了。"
    "今天还没有去医院，也没有吃药。\n"
    "请你先帮我系统整理此份病例，输出较为标准的病例格式"
)

CASES: List[Dict[str, Any]] = [
    {
        "id": "golden_vomit",
        "user_role": "veterinarian",
        "title": "指令一：金毛呕吐+鸡骨头（整理和诊断）",
        "expect_concrete": True,
        "expect_sections": ["病例整理", "问题列表", "检查与治疗方案", "风险提示"],
        "expect_case_fields": ["基本信息", "主诉", "现病史", "既往史"],
        "forbid_phrases": ["个体信号", "红旗信号"],
        "checklist_keys": [
            "case_organize",
            "case_fields",
            "problem_list",
            "differential",
            "risk_emergency",
            "workup_plan",
            "risk_section",
            "breed_hint",
            "no_ai_jargon",
        ],
        "question": _GOLDEN,
    },
    {
        "id": "flutd_organize",
        "user_role": "veterinarian",
        "title": "指令二：英短排尿困难（标准病例格式）",
        "expect_concrete": True,
        "expect_sections": ["病例整理", "问题列表", "风险提示"],
        "expect_case_fields": ["基本信息", "主诉", "现病史"],
        "forbid_phrases": ["个体信号", "红旗信号"],
        "checklist_keys": [
            "emergency_front",
            "case_organize",
            "case_fields",
            "problem_list",
            "workup_detail",
            "risk_section",
            "no_ai_jargon",
        ],
        "question": _FLUTD,
    },
    {
        "id": "bulldog",
        "user_role": "veterinarian",
        "title": "附加：法斗剧烈运动（品种特异化）",
        "expect_concrete": False,
        "forbid_required_sections": True,
        "expect_keywords": ["短吻", "热", "运动", "呼吸"],
        "checklist_keys": ["brachy_keywords", "no_forced_triad"],
        "question": (
            "3岁法斗（法国斗牛犬）公犬，已绝育，体重12kg。"
            "主人想每天带它剧烈跑步或追球1小时，最近热天遛弯就张口喘、不愿走。"
            "从兽医角度评估运动建议与风险边界，并说明与普通中型犬的差异。"
        ),
    },
    # --- 宠主对照：相同查询，验收通俗就医口吻，不套用兽医 POMR ---
    {
        "id": "golden_vomit_owner",
        "user_role": "pet_owner",
        "title": "对照：金毛呕吐+鸡骨头（宠主同问）",
        "expect_owner_tone": True,
        "forbid_required_sections": True,
        "forbid_phrases": ["个体信号", "红旗信号"],
        "checklist_keys": [
            "owner_seek_care",
            "owner_no_case_triad",
            "differential",
            "no_ai_jargon",
        ],
        "question": _GOLDEN,
    },
    {
        "id": "flutd_organize_owner",
        "user_role": "pet_owner",
        "title": "对照：英短排尿困难（宠主同问）",
        "expect_owner_tone": True,
        "forbid_required_sections": True,
        "forbid_phrases": ["个体信号", "红旗信号"],
        "checklist_keys": [
            "owner_seek_care",
            "owner_no_case_triad",
            "no_ai_jargon",
        ],
        "question": _FLUTD,
    },
    {
        "id": "bulldog_owner",
        "user_role": "pet_owner",
        "title": "对照：法斗剧烈运动（宠主同问）",
        "expect_owner_tone": True,
        "forbid_required_sections": True,
        "expect_keywords": ["短吻", "热", "运动", "呼吸"],
        "checklist_keys": ["brachy_keywords", "owner_seek_care", "owner_no_case_triad"],
        "question": (
            "3岁法斗（法国斗牛犬）公犬，已绝育，体重12kg。"
            "主人想每天带它剧烈跑步或追球1小时，最近热天遛弯就张口喘、不愿走。"
            "从兽医角度评估运动建议与风险边界，并说明与普通中型犬的差异。"
        ),
    },
]


def _default_api_key() -> str:
    """解析默认 API 密钥。"""
    keys_path = _AGENT_API / "keys.txt"
    if keys_path.exists():
        for line in keys_path.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if line and not line.startswith("#"):
                return line
    return os.getenv("AGENT_API_KEY") or "sk-petmind-default-key-2026"


def _has_any(text: str, words: List[str]) -> bool:
    """判断文本是否命中任一关键词。"""
    return any(w in text for w in words)


def _eval_checklist(case: Dict[str, Any], answer: str) -> Dict[str, str]:
    """对照清单逐项判断回答是有、无还是部分覆盖。"""
    a = answer or ""
    out: Dict[str, str] = {}
    for key in case.get("checklist_keys") or []:
        if key == "case_organize":
            out[key] = "有" if "病例整理" in a else "无"
        elif key == "case_fields":
            fields = case.get("expect_case_fields") or ["基本信息", "主诉", "现病史", "既往史"]
            hit = sum(1 for f in fields if f in a)
            if hit == len(fields):
                out[key] = "有"
            elif hit > 0:
                out[key] = "部分"
            else:
                out[key] = "无"
        elif key == "problem_list":
            out[key] = "有" if "问题列表" in a else "无"
        elif key == "differential":
            out[key] = (
                "有"
                if _has_any(a, ["鉴别", "鉴别诊断", "初步诊断", "高度怀疑", "异物", "梗阻", "胰腺炎", "FLUTD", "尿路梗阻"])
                else "无"
            )
        elif key == "risk_emergency":
            out[key] = "有" if _has_any(a, ["急诊", "紧急", "立即", "尽快就医", "禁食", "紧急处理"]) else "无"
        elif key == "workup_plan":
            out[key] = (
                "有"
                if ("检查与治疗" in a or _has_any(a, ["X光", "影像", "血常规", "超声", "检查", "治疗"]))
                else "无"
            )
        elif key == "risk_section":
            out[key] = "有" if "风险提示" in a else "无"
        elif key == "no_ai_jargon":
            bad = case.get("forbid_phrases") or ["个体信号", "红旗信号"]
            out[key] = "有" if not any(p in a for p in bad) else "无"
        elif key == "breed_hint":
            out[key] = "有" if _has_any(a, ["金毛", "品种", "易发", "高发"]) else "部分"
        elif key == "emergency_front":
            # 急症相关：优先看「检查与治疗」段内的紧急处理；否则全文关键词
            out[key] = (
                "有"
                if ("紧急处理" in a or _has_any(a, ["急诊", "紧急", "梗阻", "急症", "立即", "尽快"]))
                else "无"
            )
        elif key == "workup_detail":
            out[key] = (
                "有"
                if _has_any(a, ["检查", "治疗", "导尿", "影像", "血检", "住院", "方案"])
                else "无"
            )
        elif key == "brachy_keywords":
            out[key] = "有" if _has_any(a, ["短吻", "热", "运动", "呼吸", "张口呼吸", "热耐受"]) else "无"
        elif key == "no_forced_triad":
            forced = all(s in a for s in ("病例整理", "问题列表", "检查与治疗方案"))
            out[key] = "有" if not forced else "无"  # 有 = 满足「未强制三件套」
        elif key == "owner_seek_care":
            out[key] = (
                "有"
                if _has_any(a, ["就医", "兽医", "医院", "何时必须就医", "立即就医", "尽快就医"])
                else "无"
            )
        elif key == "owner_no_case_triad":
            forced = all(s in a for s in ("病例整理", "问题列表", "检查与治疗方案"))
            out[key] = "有" if not forced else "无"
        else:
            out[key] = "无"
    return out


def _style_checks(case: Dict[str, Any], answer: str) -> List[str]:
    """检查回答文风是否符合约定。"""
    issues: List[str] = []
    role = case.get("user_role") or "veterinarian"
    for sec in case.get("expect_sections") or []:
        if sec not in answer:
            issues.append(f"终答缺少分节关键词: {sec}")
    for field in case.get("expect_case_fields") or []:
        if field not in answer:
            issues.append(f"病例整理缺少字段: {field}")
    for phrase in case.get("forbid_phrases") or []:
        if phrase in answer:
            issues.append(f"终答含禁用 AI 风用词: {phrase}")
    if case.get("forbid_required_sections"):
        if all(s in answer for s in ("病例整理", "问题列表", "检查与治疗方案")):
            issues.append("非诊断/整理意图不应强制完整病例三件套")
    if case.get("expect_owner_tone"):
        if not _has_any(answer or "", ["就医", "兽医", "医院", "何时必须就医"]):
            issues.append("宠主终答缺少就医/兽医相关提醒")
    kws = case.get("expect_keywords") or []
    if kws and not any(k in answer for k in kws):
        issues.append(f"终答未命中任一特异化关键词: {kws}")
    return issues


def _parse_sse_block(block: str) -> Optional[Dict[str, Any]]:
    """解析一块 SSE 数据。"""
    data_lines = []
    for line in block.splitlines():
        if line.startswith("data:"):
            data_lines.append(line[5:].lstrip())
    if not data_lines:
        return None
    payload = "\n".join(data_lines).strip()
    if not payload or payload == "[DONE]":
        return {"done": True}
    try:
        return json.loads(payload)
    except json.JSONDecodeError:
        return {"raw": payload}


def _chat_stream(
    *,
    base_url: str,
    api_key: str,
    question: str,
    max_tokens: int,
    timeout_s: float,
    user_role: str = "veterinarian",
) -> Tuple[str, List[Dict[str, Any]]]:
    """发起聊天流式请求并收集事件。"""
    url = base_url.rstrip("/") + "/v1/chat/completions"
    body = {
        "model": "agent-moe",
        "stream": True,
        "temperature": 0.3,
        "max_tokens": max_tokens,
        "user_role": user_role,
        "messages": [{"role": "user", "content": question}],
    }
    headers = {
        "Authorization": f"Bearer {api_key}",
        "Content-Type": "application/json",
        "Accept": "text/event-stream",
    }
    events: List[Dict[str, Any]] = []
    answer_parts: List[str] = []
    with httpx.Client(
        timeout=httpx.Timeout(connect=10, read=timeout_s, write=30, pool=30),
        trust_env=False,
    ) as client:
        with client.stream("POST", url, headers=headers, json=body) as resp:
            if resp.status_code >= 400:
                err = resp.read().decode("utf-8", errors="replace")
                raise RuntimeError(f"HTTP {resp.status_code}: {err[:800]}")
            buf = ""
            for chunk in resp.iter_text():
                buf += chunk
                while "\n\n" in buf:
                    block, buf = buf.split("\n\n", 1)
                    obj = _parse_sse_block(block)
                    if not obj:
                        continue
                    if obj.get("done"):
                        events.append({"type": "done"})
                        continue
                    events.append(obj)
                    status = obj.get("agent_status")
                    choices = obj.get("choices") or []
                    content = ""
                    if choices:
                        content = ((choices[0].get("delta") or {}).get("content")) or ""
                    if status == "streaming" and content:
                        answer_parts.append(content)
    return "".join(answer_parts), events


def _render_report(
    case: Dict[str, Any],
    *,
    question: str,
    answer: str,
    events: List[Dict[str, Any]],
    issues: List[str],
    checklist: Dict[str, str],
    base_url: str,
) -> str:
    """把本次运行结果写成报告。"""
    lines: List[str] = []
    lines.append("# Vet Eval Live Report")
    lines.append("")
    lines.append(f"- 时间: {datetime.now().isoformat(timespec='seconds')}")
    lines.append(f"- case_id: {case['id']}")
    lines.append(f"- title: {case['title']}")
    lines.append(f"- base_url: {base_url}")
    lines.append(f"- user_role: {case.get('user_role') or 'veterinarian'}")
    lines.append("- legacy_concrete_gate: removed; classification now uses the D1-D8 LLM module")
    lines.append("")
    lines.append("## 评测清单勾选")
    lines.append("")
    for k, v in checklist.items():
        lines.append(f"- {k}: **{v}**")
    lines.append("")
    lines.append("## 请求问题")
    lines.append("")
    lines.append("```text")
    lines.append(question)
    lines.append("```")
    lines.append("")
    lines.append("## 最终答案")
    lines.append("")
    lines.append(answer or "_（空）_")
    lines.append("")
    lines.append("## 风格验收")
    lines.append("")
    if issues:
        lines.extend(f"- FAIL: {x}" for x in issues)
    else:
        lines.append("- PASS")
    lines.append("")
    lines.append(f"- SSE events: {len(events)}")
    lines.append("")
    return "\n".join(lines)


def parse_args() -> argparse.Namespace:
    """解析命令行参数并返回配置。"""
    p = argparse.ArgumentParser(description="兽医评测对照 live")
    p.add_argument("--base-url", default=os.getenv("AGENT_BASE_URL") or "http://127.0.0.1:8146")
    p.add_argument("--api-key", default=None)
    p.add_argument("--case", action="append")
    p.add_argument("--max-tokens", type=int, default=2500)
    p.add_argument("--timeout", type=float, default=420.0)
    p.add_argument("--out-dir", default=None)
    p.add_argument(
        "--fill-docs",
        action="store_true",
        help="用 checklist 回填 docs/eval_report_vet_student_comparison.md",
    )
    return p.parse_args()


def _fill_eval_doc(checklist_all: Dict[str, Dict[str, str]], summary_rows: List[str], report_dir: Path) -> None:
    """把评测字段填进文档结构。"""
    doc = _DOCS / "eval_report_vet_student_comparison.md"
    if not doc.exists():
        print(f"[warn] eval doc missing: {doc}")
        return

    g = checklist_all.get("golden_vomit") or {}
    f = checklist_all.get("flutd_organize") or {}
    b = checklist_all.get("bulldog") or {}

    def cell(d: Dict[str, str], k: str) -> str:
        """渲染表格单元格。"""
        return d.get(k, "无")

    live_section = f"""### 3.1 指令一 — 金毛 + 鸡骨头（整理和诊断）

| 检查项 | 上一版 PetMind | Deepseek（评估） | GPT（评估） | **本周 PetMind（live）** |
| --- | --- | --- | --- | --- |
| 病例整理 | 无 | 有 | 有 | **{cell(g, 'case_organize')}** |
| 临床问题列表（POMR） | 无（未作为分节） | 有 | 有 | **{cell(g, 'problem_list')}** |
| 初步鉴别 / 诊断方向 | 有（结论导向） | 有 | 有 | **{cell(g, 'differential')}** |
| 风险 / 急诊提示 | 有 | 有 | 有 | **{cell(g, 'risk_emergency')}** |
| 检查与治疗（轻重缓急） | 有（偏行动建议） | 有 | 更详尽 | **{cell(g, 'workup_plan')}** |
| 品种相关提示 | 有 | — | — | **{cell(g, 'breed_hint')}** |

### 3.2 指令二 — 英短排尿困难（标准病例格式）

| 检查项 | 上一版 PetMind | Deepseek（评估） | GPT（评估） | **本周 PetMind（live）** |
| --- | --- | --- | --- | --- |
| 急症提示前置 / 显著 | 有（√） | — | — | **{cell(f, 'emergency_front')}** |
| 病例整理 | 有 | 有（术语更严） | 有 | **{cell(f, 'case_organize')}** |
| 问题列表 | **无** | 有 | 有 | **{cell(f, 'problem_list')}** |
| 检查与治疗详尽度 | 不足 | 较详 | 更详 + 用药具体 | **{cell(f, 'workup_detail')}** |

### 3.3 附加 — 法斗剧烈运动（品种特异化）

| 检查项 | 期望 | **本周 PetMind（live）** |
| --- | --- | --- |
| 短吻 / 热耐受 / 运动或呼吸边界 | 应出现 | **{cell(b, 'brachy_keywords')}** |
| 不强制完整病例三件套 | 应满足 | **{cell(b, 'no_forced_triad')}** |
"""

    pass_n = sum(1 for r in summary_rows if ": PASS" in r)
    fail_n = sum(1 for r in summary_rows if ": FAIL" in r or ": ERROR" in r)
    total = pass_n + fail_n

    metrics = f"""### 4.2 Live 分节命中率

报告目录：`{report_dir.as_posix()}`

| 用例 | 期望分节/关键词 | 结果 | 报告目录 |
| --- | --- | --- | --- |
| golden_vomit | 病例整理、问题列表、检查与治疗方案 | {"PASS" if any("golden_vomit: PASS" in r for r in summary_rows) else "FAIL"} | `{report_dir.name}` |
| flutd_organize | 病例整理、问题列表 | {"PASS" if any("flutd_organize: PASS" in r for r in summary_rows) else "FAIL"} | `{report_dir.name}` |
| bulldog | 短吻/热/运动/呼吸；非强制三件套 | {"PASS" if any("bulldog: PASS" in r for r in summary_rows) else "FAIL"} | `{report_dir.name}` |
| **合计 PASS** | — | **{pass_n}/{total}** | — |
"""

    improved = (
        cell(g, "case_organize") == "有"
        and cell(g, "problem_list") == "有"
        and cell(f, "problem_list") == "有"
    )
    conclusion = f"""## 5. 结论

- Live 合计：**{pass_n}/{total} PASS**；详细产物见 `{report_dir.as_posix()}`。
- 相对上一版 PetMind：指令一病例整理 **{cell(g, 'case_organize')}**、问题列表 **{cell(g, 'problem_list')}**；指令二问题列表 **{cell(f, 'problem_list')}**（上一版评估为「无」）。
- 法斗特异化关键词：**{cell(b, 'brachy_keywords')}**；未强制病例三件套：**{cell(b, 'no_forced_triad')}**。
- RAG 二级类限域客观指标：药学 top-5 类内命中率 0.20→1.00（lift +0.80），见 [`rag_category_indexes.md`](rag_category_indexes.md)。
- 结构化对齐评估：{"**已覆盖**兽医学生指出的「无病例整理 / 无问题列表」核心短板。" if improved else "部分检查项未完全命中，见上表与 case 报告。"}
"""

    text = doc.read_text(encoding="utf-8")
    # Replace sections 3.1-3.3, 4.2, 5 by markers
    import re

    text = re.sub(
        r"### 3\.1 指令一[\s\S]*?(?=## 4\. 客观指标)",
        live_section + "\n",
        text,
        count=1,
    )
    text = re.sub(
        r"### 4\.2 Live 分节命中率[\s\S]*?(?=## 5\. 结论)",
        metrics + "\n",
        text,
        count=1,
    )
    text = re.sub(
        r"## 5\. 结论[\s\S]*?(?=## 6\. 附录)",
        conclusion + "\n",
        text,
        count=1,
    )
    # clear pending leftover in section 5 template if any
    doc.write_text(text, encoding="utf-8")
    print(f"[ok] filled {doc}")


def main() -> None:
    """脚本入口，解析参数并执行主流程。"""
    args = parse_args()
    api_key = args.api_key or _default_api_key()
    base = args.base_url.rstrip("/")

    try:
        with httpx.Client(timeout=5.0, trust_env=False) as c:
            r = c.get(base + "/health")
            r.raise_for_status()
    except Exception as exc:  # noqa: BLE001
        print(f"[错误] 无法连接 {base}/health：{exc}")
        raise SystemExit(2)

    # Wait until RAG warmup marks /ready (avoid empty first-answer during cold start).
    ready_deadline = time.time() + 600.0
    while time.time() < ready_deadline:
        try:
            with httpx.Client(timeout=5.0, trust_env=False) as c:
                rr = c.get(base + "/ready")
                if rr.status_code == 200 and (rr.json() or {}).get("ready"):
                    warm = (rr.json() or {}).get("warmup") or {}
                    print(f"[ready] warmup={warm}")
                    break
        except Exception:  # noqa: BLE001
            pass
        time.sleep(2.0)
    else:
        print("[warn] /ready 未在超时内变为 true，继续跑用例（可能遇冷启动空答）")

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = Path(args.out_dir) if args.out_dir else (_THIS.parent / "reports" / f"vet_eval_{ts}")
    out_dir.mkdir(parents=True, exist_ok=True)

    selected = CASES
    if args.case:
        ids = set(args.case)
        selected = [c for c in CASES if c["id"] in ids]
        if not selected:
            print(f"[错误] 无匹配 case: {args.case}")
            raise SystemExit(2)

    summary: List[str] = []
    checklist_all: Dict[str, Dict[str, str]] = {}
    all_ok = True
    for case in selected:
        print(f"\n=== RUN {case['id']}: {case['title']} ===")
        q = case["question"]
        try:
            answer, events = _chat_stream(
                base_url=base,
                api_key=api_key,
                question=q,
                max_tokens=args.max_tokens,
                timeout_s=args.timeout,
                user_role=str(case.get("user_role") or "veterinarian"),
            )
        except Exception as exc:  # noqa: BLE001
            print(f"[FAIL] 请求异常: {exc}")
            all_ok = False
            summary.append(f"- {case['id']}: ERROR ({exc})")
            checklist_all[case["id"]] = {k: "无" for k in case.get("checklist_keys") or []}
            continue

        checklist = _eval_checklist(case, answer)
        checklist_all[case["id"]] = checklist
        issues = _style_checks(case, answer)
        md = _render_report(
            case,
            question=q,
            answer=answer,
            events=events,
            issues=issues,
            checklist=checklist,
            base_url=base,
        )
        out_path = out_dir / f"case_{case['id']}.md"
        out_path.write_text(md, encoding="utf-8")
        (out_dir / f"case_{case['id']}_answer.txt").write_text(answer or "", encoding="utf-8")
        ok = not issues
        all_ok = all_ok and ok
        status = "PASS" if ok else "FAIL"
        print(f"[{status}] -> {out_path}")
        for k, v in checklist.items():
            print(f"  checklist {k}: {v}")
        for iss in issues:
            print(f"  - {iss}")
        summary.append(f"- {case['id']}: {status} ({out_path.name})")

    (out_dir / "checklist.json").write_text(
        json.dumps(checklist_all, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    (out_dir / "SUMMARY.md").write_text(
        "# Vet Eval Live Summary\n\n" + "\n".join(summary) + "\n", encoding="utf-8"
    )
    print(f"\n报告目录: {out_dir}")

    if args.fill_docs:
        _fill_eval_doc(checklist_all, summary, out_dir)

    raise SystemExit(0 if all_ok else 1)


if __name__ == "__main__":
    main()
