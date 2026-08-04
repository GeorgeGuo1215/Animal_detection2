from __future__ import annotations

import json
import os
import re
import time
from datetime import date
from typing import Any, Dict, List, Optional, Tuple

from ..context.request_context import ANIMAL_REQUIRED_TOOLS, get_request_animal_id
from ..llm.llm_client import AsyncOpenAIClient, OpenAICompatClient, extract_text
from ..tools.tool_registry import ToolRegistry
from ..concurrency import ResourceBusyError


# 模型无法自数 token，把 max_tokens 原样写进提示词不起作用，需折算成汉字预算。
# 中文正文在 DeepSeek 词表下约 1 token 承载 1.5 个汉字；再留 20% 余量给收尾段落，
# 因为撞到 max_tokens 是句中硬截断，丢掉的正是末尾的免责声明与最后一个分节。
_CHARS_PER_TOKEN = 1.5
_LENGTH_SAFETY_RATIO = 0.8
_MIN_ANSWER_CHAR_BUDGET = 300


def answer_char_budget(max_tokens: Optional[int]) -> Optional[int]:
    """把终答的 token 预算折算为提示词里可执行的汉字预算；无预算时返回 None。"""
    if not max_tokens:
        return None
    tokens = int(max_tokens)
    if tokens <= 0:
        return None
    return max(_MIN_ANSWER_CHAR_BUDGET, int(tokens * _CHARS_PER_TOKEN * _LENGTH_SAFETY_RATIO))


def _length_budget_instruction(char_budget: int) -> str:
    return (
        "\n\n**篇幅预算**\n"
        f"- 全文（含分节标题、列表与参考来源）控制在 {char_budget} 字以内。\n"
        "- 这是硬预算：超出会被系统在句子中间直接切断，末尾的分节与免责声明会整段丢失。\n"
        "- 篇幅紧张时，优先保证分节结构完整和结尾写完，压缩各节内部的展开细节，不要删减分节。\n"
    )


_CITATION_INSTRUCTION = (
    "\n\n**引用规范**\n"
    "回答时，请在相关段落末尾标注来源。格式示例：\n"
    "- 中文书籍：（《书名》第 X 页）\n"
    "- 英文书籍：(*Book Title*, p. X)\n"
    "来源信息可从工具返回的 source_path 和文本中的 '--- Page N of M ---' 标记推断。\n"
    "在回答末尾汇总所有引用，使用如下格式：\n"
    "> **参考来源**\n"
    "> - 《书名》第 X 页\n"
    "> - *English Book Title*, p. X\n"
)

_WEB_CITATION_INSTRUCTION = (
    "\n\n**网络搜索结果引用规范**\n"
    "当回答中使用了 web_search 工具返回的信息时：\n"
    "1. 在引用处标注来源编号，如 [1]、[2]\n"
    "2. 在回答末尾的「参考来源」区列出所有网络来源，保留完整 URL，格式如下：\n"
    "> **参考来源（网络）**\n"
    "> 1. [文章标题](https://example.com/url)\n"
    "> 2. [文章标题](https://example.com/url)\n\n"
    "来源的 title 和 url 可从 web_search 工具返回的 results 中提取。\n"
    "URL 必须从工具结果逐字符原样复制；禁止翻译、纠错、补全、缩短、解码后重编码，"
    "也禁止替换百分号编码中的任何字符。不可编造链接。如果工具未返回 URL，则只标注标题。\n"
)


_VET_EVIDENCE_LAYERING_INSTRUCTION = (
    "\n\n**兽医证据分层规范**\n"
    "当回答涉及疾病、症状、诊断、治疗、用药、手术或病例时，必须执行证据分层：\n"
    "1. 先给出「循证兽医学直接证据」：仅写入已被临床研究、教科书、权威指南或病例报告直接支持的事实。\n"
    "2. 再给出「临床经验与推断」：这一部分只能基于临床经验或跨物种推断，必须明确标注为推断或参考。\n"
    "3. 如果检索结果缺乏直接证据，必须明确说明『当前直接证据有限』，然后再谨慎提供临床参考。\n"
    "4. 遇到病因、治疗方案、药物剂量、手术适应证、预后等高风险信息时，优先使用『可能』『常见于』"
    "『建议结合临床评估』等表述，禁止过度确定化。\n"
    "5. 优先使用如下结构（用加粗文字分节）：\n"
    "   - **临床直接证据**\n"
    "   - **临床经验与推断**\n"
    "   - **鉴别诊断与进一步检查建议**\n"
    "   - **证据不足与注意事项**\n"
)

_VET_CONCRETE_CASE_INSTRUCTION = (
    "\n\n**兽医病例工作流答风（用户明确要求病例整理，或要求对病例做完整诊断）**\n"
    "仅在此类意图下启用。你是面向执业兽医的 AI 助手，用语简明清晰，像病历与诊疗记录，不要写 AI 套话，也不要自称『同事』。\n"
    "禁止使用难懂或翻译腔用词与口号式标签；病例字段只用「基本信息、主诉、现病史、既往史」等常规病历用语，"
    "不要写「信号」类标签或「痛点、闭环、赋能」等套话。\n"
    "用**加粗文字**分节（禁止 Markdown 标题 # ## ###），按下列顺序输出，不要增删分节名：\n"
    "1. **病例整理**（按条件归纳、分点叙述；未知项写「未提供」，勿编造）\n"
    "   - **基本信息**：物种/品种、年龄、性别、绝育/去势、体重（若有）等。\n"
    "   - **主诉**：一句到两句，简明干练（症状 + 大致病程），不要展开时间线细节。\n"
    "   - **现病史**：按时间与症状分点（起病、演变、呕吐/排便/饮水/精神、诱因等）。\n"
    "   - **既往史**：既往疾病、慢病等；若材料涉及，再开子点写清："
    "疫苗史、驱虫史、药物过敏史、手术史、近期用药/就医情况、饮食与环境相关史。"
    "有信息才写子点；没有则在既往史中一句带过「未提及过敏/手术等」。\n"
    "2. **问题列表**：这是 POMR 的核心，**不是**向宠主追问的问题清单，"
    "**也不是**把怀疑的病名直接当成「问题」。\n"
    "   每条「问题」只能写**已明确得到的临床表现或指标异常**"
    "（如：呕吐、腹泻、腹痛、发热、脱水、异物梗阻影像学提示、超敏反应蛋白升高、白细胞升高等）。\n"
    "   格式建议：`问题N：<症状或异常指标>`，其下再用短句写"
    "「由此导向怀疑… / 需鉴别…」，把疾病假设（如胰腺炎、异物梗阻、胃肠炎）放在鉴别里，"
    "**不要**写成「问题1：急性胰腺炎」。\n"
    "   优先按器官系统归类；材料里没有的体征或检验值一律不要编造。\n"
    "3. **检查与治疗方案**（若用户只要『整理病例/病例格式』可写得更短，但仍建议保留鉴别后的下一步要点）\n"
    "   本节内部严格按顺序：\n"
    "   (1) **紧急处理**：先判断是否急症。若是，写接诊/院内处置优先级"
    "（如 NPO、补液、镇痛、影像与实验室优先次序、监护要点）；"
    "若不是急症，用一两句说明『目前不像需要立刻急诊处置，可按下列计划安排』，再往下写。"
    "面向兽医用户：不要写『请立即就医/带去医院/路上注意点』等宠主话术。\n"
    "   (2) **检查方案**：按优先级列出体格检查与辅助检查。\n"
    "   (3) **治疗方案**：原则、用药与监测边界、复诊/再评估指征；证据分层"
    "（先写临床直接证据，再写经验推断并标注）。\n"
    "4. **风险提示**（必须放在全文最后，在参考文献之前）\n"
    "   用短列表写临床风险边界：主要并发症、恶化/红旗征象、关键用药的物种毒性与处方边界、监测要点。"
    "不要写『勿自行给人药』『需线下执业兽医确认』『请立即就医』等宠主口吻。语气克制，像诊疗记录。\n"
    "若用户只是就某个具体问题征求意见（如运动建议、用药咨询、风险边界），"
    "**不要**强行套用本结构，按问题本身作答即可。\n"
)

# 具体宠物信号：年龄 / 常见品种 / 性别 / 绝育 / 体重
_PET_SIGNAL_RE = re.compile(
    r"("
    r"\d+\s*岁|\d+\s*(?:year|yr|month|mo)s?|"
    r"月龄|个月大|"
    r"英短|美短|加短|蓝猫|布偶|波斯|暹罗|缅因|橘猫|狸花|"
    r"斗牛|法斗|英斗|巴哥|柴犬|柯基|金毛|拉布拉多|泰迪|比熊|德牧|哈士奇|边牧|雪纳瑞|吉娃娃|"
    r"british\s*shorthair|persian|ragdoll|siamese|maine\s*coon|"
    r"bulldog|pug|corgi|retriever|poodle|shepherd|"
    r"公猫|母猫|公犬|母犬|公狗|母狗|"
    r"(?:^|[^\w])(?:公|母|雄|雌)(?:[^\w]|$)|"
    r"male|female|"
    r"绝育|去势|阉割|未绝育|spay(?:ed)?|neuter(?:ed)?|"
    r"\d+(?:\.\d+)?\s*(?:kg|公斤|斤)"
    r")",
    re.IGNORECASE,
)

# 具体病情信号：症状 / 时间线 / 既往与处置
_CONDITION_SIGNAL_RE = re.compile(
    r"("
    r"疾病|症状|诊断|治疗|手术|用药|药物|剂量|病例|感染|炎症|肿瘤|癌|骨折|"
    r"呕吐|干呕|腹泻|软便|便秘|发烧|发热|咳嗽|抽搐|中毒|过敏|寄生虫|"
    r"疫苗|免疫|麻醉|驱虫|尿血|血尿|尿频|尿少|排尿|猫砂|蹲很久|尿不出|"
    r"膀胱|结石|食欲|不爱吃|精神|趴着|舔下面|喘气|呼吸|跛行|伤口|"
    r"今天|昨天|早上|昨晚|十几分钟|持续|最近|"
    r"吃药|去医院|就诊|既往|病史|"
    r"disease|symptom|diagnos|treatment|surgery|medication|dose|infection|"
    r"tumor|cancer|fracture|vomit|diarrhea|fever|seizure|poison|parasite|vaccine|"
    r"hematuria|stranguria|dysuria|anorexi|letharg"
    r")",
    re.IGNORECASE,
)


# 用户明确要求「整理病例 / 统一病例格式」
_CASE_ORGANIZE_INTENT_RE = re.compile(
    r"("
    r"病历|病例|病例整理|整理病例|整理病历|病历整理|病例格式|统一的?病例|问题列表|"
    r"输出.*病例|请对以上病例.*整理|对以上病例做出整理|SOAP|"
    r"怎么整理|应该怎么整理|整理和诊断|病例应该怎么|"
    r"系统整理.*病例|标准的?病例格式|"
    r"organize\s+(the\s+)?case|case\s+summary\s+format"
    r")",
    re.IGNORECASE,
)

# 用户要求对病例做完整诊断（含鉴别/诊疗全过程；病例整理是其中环节）
_DIAGNOSIS_INTENT_RE = re.compile(
    r"("
    r"做出诊断|进行诊断|完整诊断|诊断一下|请诊断|帮我诊断|"
    r"对以上病例.*诊断|请对以上.*诊断|鉴别诊断|"
    r"怎么诊断|应该怎么.*诊断|整理和诊断|和诊断|"
    r"完整诊疗|诊疗意见|临床诊断|"
    r"diagnos(?:e|is|tic\s+workup)"
    r")",
    re.IGNORECASE,
)


def _is_medical_query(query: str) -> bool:
    """Simple heuristic to detect medical/clinical queries for pets."""
    return bool(_CONDITION_SIGNAL_RE.search(query or ""))


_WEB_SEARCH_TOOL = "mcp.web_search.web_search"
_VITALS_SQL_ONLY = frozenset({"vitals.summary", "sql.search"})


def ensure_rag_and_web_tool_steps(
    tool_steps: List[Dict[str, Any]],
    *,
    visible_tools: List[str],
    query: str,
    rag_query: Optional[str] = None,
) -> List[Dict[str, Any]]:
    """For medical queries, ensure both rag.search and web_search are planned when available.

    Skips when either tool is unavailable, query is non-medical, or the plan is
    purely vitals/sql (personal-data path). Does not truncate; caller may cap steps.
    """
    visible = set(visible_tools or [])
    if "rag.search" not in visible or _WEB_SEARCH_TOOL not in visible:
        return list(tool_steps or [])
    if not _is_medical_query(query):
        return list(tool_steps or [])

    steps = [s for s in (tool_steps or []) if isinstance(s, dict)]
    names = {str(s.get("tool_name") or "") for s in steps}
    if names and names <= _VITALS_SQL_ONLY:
        return steps

    out = list(steps)
    if "rag.search" not in names:
        out.insert(
            0,
            {
                "type": "tool",
                "tool_name": "rag.search",
                "arguments": {"query": (rag_query or query)},
                "note": "paired with web_search for medical synthesis",
            },
        )
    if _WEB_SEARCH_TOOL not in names:
        out.append(
            {
                "type": "tool",
                "tool_name": _WEB_SEARCH_TOOL,
                "arguments": {"query": query, "max_results": 5},
                "note": "paired with rag.search for medical synthesis",
            }
        )
    return out


def has_concrete_case_narrative(query: str) -> bool:
    """True when text looks like a concrete pet + condition narrative (not intent)."""
    text = (query or "").strip()
    if len(text) < 40:
        return False
    return bool(_PET_SIGNAL_RE.search(text) and _CONDITION_SIGNAL_RE.search(text))


def is_concrete_vet_case(query: str) -> bool:
    """Gate for 病例整理/问题列表/检查与治疗 结构化答风.

    Requires a concrete case narrative PLUS one of:
    1) explicit request to organize the case / output a case format; or
    2) request for a full diagnosis / clinical workup on the case.
    Focused asks (e.g. exercise advice only) must NOT trigger this.
    """
    text = (query or "").strip()
    if not has_concrete_case_narrative(text):
        return False
    return bool(_CASE_ORGANIZE_INTENT_RE.search(text) or _DIAGNOSIS_INTENT_RE.search(text))


def build_solve_prompt(
    user_role: str = "pet_owner",
    has_web_search: bool = False,
    query: str = "",
    max_tokens: Optional[int] = None,
) -> str:
    """Build role-aware system prompt for the solve/generation stage.

    `max_tokens` 是该次生成的实际 token 预算；传入后会折算成汉字预算写进提示词，
    让模型主动分配篇幅而不是被硬截断。省略则不附加篇幅约束。
    """
    today = date.today().isoformat()
    concrete_case = user_role == "veterinarian" and is_concrete_vet_case(query)

    if user_role == "veterinarian":
        prompt = (
            f"你是面向执业兽医的 AI 临床助手（不是兽医同事、也不扮演真人医生）。今天是 {today}。\n"
            "你拥有以下能力：知识库检索、网络搜索与成分分析、营养与运动计划制定。\n\n"
            "回答要求：\n"
            "- 使用专业兽医学术中文，术语准确，逻辑严谨；以 AI 助手身份提供结构化临床参考\n"
            "- 回答需结构化呈现，用**加粗文字**作为分节标记，配合列表和段落组织内容\n"
            "- 禁止使用 Markdown 标题语法（# ## ### 等），分节一律用加粗文字代替\n"
            "- 引用具体来源（书名、页码、文献），不可编造\n"
            "- 如果证据不足，明确说明当前知识库尚无定论，建议查阅更多文献或结合临床评估\n"
            "- 尽可能给出药物剂量范围、实验室指标参考值、鉴别诊断等专业信息\n"
            "- 急症写处置优先级、检查与监护要点；用药写物种毒性、相互作用与处方/监测边界\n"
            "- **禁止宠主话术**：不要写『请立即就医』『带去医院』『尽快线下就诊』"
            "『勿自行给人药/布洛芬』『需线下执业兽医确认』『强烈建议联系兽医』等面向宠主的提醒\n"
            "- 终答不要自称『同事』或『作为您的同事』；保持 AI 助手专业口吻\n"
            "- 如果工具返回 INSUFFICIENT_DATA，请明确告知需补充更多检查结果或病史\n"
            "- 如果工具返回 FEEDING_INQUIRY_NEEDED，请主动询问近期饮食与给药情况\n"
            "- 如果工具返回了 sql.search 的表格数据，只使用其中出现的字段与数值，不要编造行内不存在的数据\n"
        )
    else:
        prompt = (
            f"你是一位热情的宠物健康顾问，擅长用生动有趣的方式回答关于宠物养护和健康的问题。今天是 {today}。\n"
            "你拥有以下能力：知识库检索、网络搜索与成分分析、营养与运动计划制定。\n\n"
            "回答要求：\n"
            "- 用通俗易懂、活泼亲切的中文回答\n"
            "- 适当使用比喻、趣闻和实用小贴士让内容更有吸引力\n"
            "- 不需要过于学术化，但信息要准确\n"
            "- 用**加粗文字**分节，配合列表组织内容，不要过度分节\n"
            "- 禁止使用 Markdown 标题语法（# ## ### 等），分节一律用加粗文字代替\n"
            "- 如果证据不足，坦诚告知并建议咨询兽医\n"
            "- 对疾病症状类问题采用对话式分诊：当用户提供的信息不足且未明确报告当前红旗时，"
            "先回答本轮最需要观察的事项，并提出 3~6 个能改变风险判断的关键问题；不要仅因鉴别诊断中"
            "存在严重疾病，就把罕见或最坏情况写成当前最可能结论、渲染危急氛围或无条件要求立即就医\n"
            "- 在上述信息不足的首轮，可说明常见原因和风险类别，但不要主动点名、展开尚无个体证据的罕见"
            "严重疾病；优先描述宠物主实际能观察的变化。用户明确询问鉴别诊断、已有相关证据或报告红旗时，"
            "仍应正常说明疾病级风险，不得隐瞒\n"
            "- 首轮可给出短时观察方法和清晰的升级处置阈值；用户后续补充信息后，再结合新信息给出更完整的"
            "鉴别方向、行动建议和就医时机，不要机械重复同一组追问\n"
            "- 若用户已经报告呼吸困难、意识异常、持续倒地、无法排尿、持续大量出血等明确当前红旗，"
            "必须在本轮直接说明紧急程度和就医行动，不得为了追问而延误；是否紧急只依据用户已报告的表现，"
            "不能把『某症状可能由严重疾病引起』等同于患者已经处于急症\n"
            "- 如果工具返回 INSUFFICIENT_DATA，请用友好的方式引导用户提供更多信息（如拍照产品成分表）\n"
            "- 如果工具返回 FEEDING_INQUIRY_NEEDED，请用轻松的方式询问宠物今天是否已经进食\n"
            "- 如果工具返回了 sql.search 的表格数据，只使用其中出现的字段与数值，不要编造行内不存在的数据\n"
        )

    prompt += _CITATION_INSTRUCTION

    if has_web_search:
        prompt += _WEB_CITATION_INSTRUCTION
        prompt += (
            "\n\n**使用工具结果的优先级规则**\n"
            "你同时收到了本地知识库（rag.search）和网络搜索（web_search）的结果。\n"
            "**必须遵守以下优先级**：\n"
            "1. 如果本地知识库的结果与用户问题**直接相关**（内容能回答问题），以知识库为主，网络搜索作为补充。\n"
            "2. 如果本地知识库的结果与用户问题**无关或不足**（内容是其他话题），"
            "**必须以网络搜索结果为主**来正面回答用户问题，不要说『知识库没有相关信息』。\n"
            "3. **禁止**在网络搜索结果已能回答问题的情况下，仍然回答『无法找到相关信息』。\n"
            "4. 如果网络搜索结果也不足，才可说明信息有限，但仍需基于已有结果尽力回答。\n"
        )

    if concrete_case:
        prompt += _VET_CONCRETE_CASE_INSTRUCTION
    elif user_role == "veterinarian" and _is_medical_query(query):
        prompt += _VET_EVIDENCE_LAYERING_INSTRUCTION

    char_budget = answer_char_budget(max_tokens)
    if char_budget is not None:
        prompt += _length_budget_instruction(char_budget)

    return prompt


def _safe_json_loads(text: str) -> Tuple[Optional[Dict[str, Any]], str]:
    """
    Best-effort JSON extraction for LLM outputs.
    Returns (obj, error_message).
    """
    text = (text or "").strip()
    if not text:
        return None, "empty response"
    try:
        return json.loads(text), ""
    except Exception:
        pass

    # try to extract the first JSON object
    l = text.find("{")
    r = text.rfind("}")
    if l >= 0 and r > l:
        snippet = text[l : r + 1]
        try:
            return json.loads(snippet), ""
        except Exception as e:  # noqa: BLE001
            return None, f"json parse failed: {e}"
    return None, "json object not found"


def _planner_tool_brief(
    registry: ToolRegistry,
    allowed_tools: Optional[List[str]],
) -> List[Dict[str, Any]]:
    allowed = set(allowed_tools) if allowed_tools is not None else None
    return [
        {"name": tool.name, "description": tool.description, "input_schema": tool.input_schema}
        for tool in registry.list_tools()
        if (tool.name != "sql.search" or get_request_animal_id())
        and (allowed is None or tool.name in allowed)
    ]


def _planner_user_payload(query: str, tool_brief: List[Dict[str, Any]]) -> Dict[str, Any]:
    return {
        "query": query,
        "available_tools": tool_brief,
        "output_format": {
            "steps": [
                {
                    "type": "tool",
                    "tool_name": "<tool_name>",
                    "arguments": {"<key>": "<value>"},
                    "note": "为什么调用这个工具",
                },
                {"type": "final", "note": "最后如何组织答案"},
            ]
        },
    }


def _parse_plan_output(text: str) -> List[Dict[str, Any]]:
    obj, err = _safe_json_loads(text)
    if not obj or "steps" not in obj or not isinstance(obj["steps"], list):
        raise RuntimeError(f"Planner output is not valid JSON plan: {err}. raw={text[:800]}")
    return obj["steps"]


def _solve_messages(
    *,
    query: str,
    plan_steps: List[Dict[str, Any]],
    tool_results: List[Dict[str, Any]],
    user_role: str,
    max_tokens: Optional[int] = None,
) -> List[Dict[str, str]]:
    has_web = any(r.get("tool_name", "").startswith("mcp.web_search") for r in tool_results)
    payload = {"query": query, "plan": plan_steps, "tool_results": tool_results}
    return [
        {
            "role": "system",
            "content": build_solve_prompt(
                user_role=user_role, has_web_search=has_web, query=query,
                max_tokens=max_tokens,
            ),
        },
        {"role": "user", "content": json.dumps(payload, ensure_ascii=False)},
    ]


class PlanAndSolveAgent:
    def __init__(self, *, registry: ToolRegistry, llm: OpenAICompatClient) -> None:
        self.registry = registry
        self.llm = llm

    @staticmethod
    def _force_rag_search_defaults(arguments: Dict[str, Any]) -> Dict[str, Any]:
        """
        Force RAG quality defaults for agent runs.

        Rationale: in practice, planners may omit critical knobs. For this project we want
        multi-route retrieval + reranking + neighbor expansion enabled by default to reduce
        retrieval misses and improve evidence quality.

        This function enforces:
        - multi_route = True
        - rerank = True
        - expand_neighbors >= 1

        It also fills a few safe defaults when missing.
        """
        args = dict(arguments or {})

        # hard-enforce key behaviors
        args["multi_route"] = True
        args["rerank"] = True
        if int(args.get("expand_neighbors") or 0) < 1:
            args["expand_neighbors"] = 1

        # sensible defaults if missing
        args.setdefault("rewrite", "template")
        args.setdefault("top_k", 5)
        args.setdefault("device", os.getenv("AGENT_WARMUP_DEVICE") or None)

        # ensure we retrieve enough candidates before rerank
        top_k = int(args.get("top_k") or 5)
        args.setdefault("rerank_candidates", max(10, top_k * 2))
        args.setdefault("rerank_batch_size", 32)
        args.setdefault("rerank_filter_overlap", float(os.getenv("RAG_OVERLAP_THRESHOLD", "0.15")))

        return args

    def plan(self, *, query: str, allowed_tools: Optional[List[str]] = None) -> List[Dict[str, Any]]:
        tool_brief = _planner_tool_brief(self.registry, allowed_tools)

        sys = (
            f"你是一个严谨的 AI Agent 规划器，采用 Plan-and-Solve。今天是 {date.today().isoformat()}。"
            "你必须输出严格 JSON，不要输出任何额外文字。"
            "你的目标是：用尽量少的步骤解决用户问题。\n"
            "工具选择：\n"
            "- 健康/医学/临床类问题：若 available_tools 同时含 rag.search 与 mcp.web_search.web_search，"
            "**应同计划调用二者**（rag.search 用英语 query；web_search 用中文 query），再综合结果；\n"
            "- 纯体征/日报类可优先 vitals.summary 或 sql.search，不必强行附加 rag/web；\n"
            "- 产品信息/价格线索可侧重 web_search；成分安全性用 ingredient_check。\n"
            "若上下文中提供了当前宠物的 animal_id，可用 sql.search 只读查询表 daily_reports（日报）；"
            "无 animal_id 时不要规划 sql.search。"
        )
        user = _planner_user_payload(query, tool_brief)

        resp = self.llm.chat(
            messages=[
                {"role": "system", "content": sys},
                {"role": "user", "content": json.dumps(user, ensure_ascii=False)},
            ],
            temperature=0.1,
            max_tokens=512,
            # if supported, nudges JSON output
            response_format={"type": "json_object"},
        )
        text = extract_text(resp)
        return _parse_plan_output(text)

    def solve(
        self,
        *,
        query: str,
        plan_steps: List[Dict[str, Any]],
        allowed_tools: Optional[List[str]] = None,
        temperature: float = 0.2,
        max_tokens: int = 768,
        user_role: str = "pet_owner",
    ) -> Tuple[str, List[Dict[str, Any]]]:
        allowed = set(allowed_tools) if allowed_tools is not None else None
        tool_results: List[Dict[str, Any]] = []

        for i, step in enumerate(plan_steps):
            stype = step.get("type")
            if stype == "tool":
                tool_name = str(step.get("tool_name") or "")
                if not tool_name:
                    continue
                if allowed is not None and tool_name not in allowed:
                    raise RuntimeError(f"Tool not allowed: {tool_name}")
                args = step.get("arguments") or {}
                if not isinstance(args, dict):
                    args = {}

                if tool_name == "rag.search":
                    args = self._force_rag_search_defaults(args)
                if tool_name == "sql.search" and not get_request_animal_id():
                    raise RuntimeError("sql.search requires request animal_id (JSON animal_id or X-Animal-Id header)")
                result = self.registry.call_sync(tool_name, args)
                tool_results.append({"step": i, "tool_name": tool_name, "arguments": args, "result": result})
            elif stype == "final":
                break
            else:
                continue

        resp = self.llm.chat(
            messages=_solve_messages(
                query=query, plan_steps=plan_steps, tool_results=tool_results, user_role=user_role,
                max_tokens=max_tokens,
            ),
            temperature=temperature,
            max_tokens=max_tokens,
        )
        answer = extract_text(resp)
        return answer, tool_results


async def execute_tool_steps(
    *,
    registry: ToolRegistry,
    plan_steps: List[Dict[str, Any]],
    allowed_tools: Optional[List[str]] = None,
) -> List[Dict[str, Any]]:
    """Execute the `tool` steps of a plan against the registry.

    Shared by AsyncPlanAndSolveAgent.solve and the MoE experts so tool dispatch
    semantics stay identical. Enforces rag.search quality defaults, the animal_id
    requirement for animal-scoped tools (sql.search / vitals.summary) and an optional
    allowed_tools whitelist. A failing tool is captured (ok=False) instead of aborting
    the whole run. Returns one entry per executed tool with result, ok and latency_ms.
    """
    allowed = set(allowed_tools) if allowed_tools is not None else None
    results: List[Dict[str, Any]] = []
    for i, step in enumerate(plan_steps):
        stype = step.get("type")
        if stype == "final":
            break
        if stype != "tool":
            continue
        tool_name = str(step.get("tool_name") or "")
        if not tool_name:
            continue
        if allowed is not None and tool_name not in allowed:
            raise RuntimeError(f"Tool not allowed: {tool_name}")
        args = step.get("arguments") or {}
        if not isinstance(args, dict):
            args = {}
        if tool_name == "rag.search":
            args = PlanAndSolveAgent._force_rag_search_defaults(args)
        if tool_name in ANIMAL_REQUIRED_TOOLS and not get_request_animal_id():
            raise RuntimeError(
                f"{tool_name} requires request animal_id (JSON animal_id or X-Animal-Id header)"
            )
        t0 = time.perf_counter()
        try:
            result = await registry.call(tool_name, args)
            ok, err = True, ""
        except ResourceBusyError as exc:
            result, ok, err = exc.as_dict(), False, str(exc)
        except Exception as exc:  # noqa: BLE001
            result, ok, err = {"error": str(exc)}, False, str(exc)
        results.append(
            {
                "step": i,
                "tool_name": tool_name,
                "arguments": args,
                "result": result,
                "ok": ok,
                "latency_ms": round((time.perf_counter() - t0) * 1000.0, 1),
                "error": err,
            }
        )
    return results


class AsyncPlanAndSolveAgent:
    """Fully async version that uses AsyncOpenAIClient and async registry.call()."""

    def __init__(self, *, registry: ToolRegistry, llm: AsyncOpenAIClient) -> None:
        self.registry = registry
        self.llm = llm

    _force_rag_search_defaults = staticmethod(PlanAndSolveAgent._force_rag_search_defaults)

    async def plan(
        self,
        *,
        query: str,
        allowed_tools: Optional[List[str]] = None,
        recorder: Optional[Any] = None,
        stage: str = "planner",
    ) -> List[Dict[str, Any]]:
        tool_brief = _planner_tool_brief(self.registry, allowed_tools)

        sys = (
            f"你是一个严谨的 AI Agent 规划器，采用 Plan-and-Solve。今天是 {date.today().isoformat()}。"
            "你必须输出严格 JSON，不要输出任何额外文字。"
            "你的目标是：准确解决用户问题；在确有帮助时应主动调用相关工具（可规划多个工具步），"
            "但要避免无谓或重复调用。\n"
            "个性化优先：当请求已带 animal_id（说明该宠物在库中有档案/体征/日报）时，"
            "凡涉及“我家这只宠物”的具体状况，应优先用 vitals.summary 获取其真实心率/呼吸/体温，"
            "或用 sql.search 查该宠物的日报/档案，不要只凭通用知识作答。\n"
            "工具选择指南：\n"
            "- 健康/医学/临床类问题：若同时有 rag.search 与 mcp.web_search.web_search，"
            "**应同计划调用二者并综合**（rag.search 英文 query；web_search 中文 query）；"
            "知识类优选，但医学问题不要只调其中一个\n"
            "- 当前请求已带 animal_id 且需查该宠物的**日报**结构化数据 → sql.search（仅表 daily_reports；"
            "参数含 database、table='daily_reports'、可选 columns/where/order_by/limit；服务端会强制按 animal_id 过滤）\n"
            "- 实时体征（HR/RR/体温）→ vitals.summary；纯体征/日报问题可优先体征工具，不必强行附加 rag/web\n"
            "- 实时网络信息 / 产品信息 / 价格线索 → mcp.web_search.web_search\n"
            "- 产品成分安全性 → mcp.web_search.ingredient_check\n"
            "- 喂食量/热量计算 → mcp.nutritional_planner.calculate_meal_plan\n"
            "- 运动计划 → mcp.nutritional_planner.generate_exercise_plan\n"
            "对于 rag.search 使用英语查询，其他工具按其 input_schema 填写参数。"
        )
        user = _planner_user_payload(query, tool_brief)

        messages = [
            {"role": "system", "content": sys},
            {"role": "user", "content": json.dumps(user, ensure_ascii=False)},
        ]
        t0 = time.perf_counter()
        resp = await self.llm.chat(
            messages=messages,
            temperature=0.1,
            max_tokens=512,
            response_format={"type": "json_object"},
        )
        text = extract_text(resp)
        if recorder is not None:
            try:
                recorder.record_llm(
                    stage=stage,
                    model=getattr(self.llm, "model", ""),
                    messages=messages,
                    output=text,
                    latency_ms=(time.perf_counter() - t0) * 1000.0,
                    usage=(resp or {}).get("usage"),
                )
            except Exception:  # noqa: BLE001
                pass
        return _parse_plan_output(text)

    async def solve(
        self,
        *,
        query: str,
        plan_steps: List[Dict[str, Any]],
        allowed_tools: Optional[List[str]] = None,
        temperature: float = 0.2,
        max_tokens: int = 768,
        user_role: str = "pet_owner",
    ) -> Tuple[str, List[Dict[str, Any]]]:
        tool_results = await execute_tool_steps(
            registry=self.registry,
            plan_steps=plan_steps,
            allowed_tools=allowed_tools,
        )

        resp = await self.llm.chat(
            messages=_solve_messages(
                query=query, plan_steps=plan_steps, tool_results=tool_results, user_role=user_role,
                max_tokens=max_tokens,
            ),
            temperature=temperature,
            max_tokens=max_tokens,
        )
        answer = extract_text(resp)
        return answer, tool_results
