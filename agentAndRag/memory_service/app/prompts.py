"""记忆提升用的中文提示词。

整个提升流程只有三次 LLM 调用，对应下面三组提示词。上游 MemoryOS 还会为每一页
额外调两次（连续性判断 + meta_info 生成），前者已被 heat.py 的启发式取代，
后者产出的信息在检索里几乎没被用到，直接砍掉。

三个提示词都要求严格 JSON 输出，并且都带一句"没有就留空"——宁可这一轮没有产出，
也不要让模型为了填满结构而编造宠物病史。
"""

from __future__ import annotations

from typing import Any, Dict, List, Sequence

SUMMARY_SYSTEM = """你是宠物健康助手的对话归档员。你的任务是把一批用户与助手的对话按主题归类，生成主题摘要。

要求：
1. 按话题把对话分组，一组一个主题；同一话题的多轮对话归为一组。
   **涉及不同宠物的对话必须分到不同组**，哪怕问的是同一件事——
   "咪咪的体重"和"豆豆的体重"是两组，不要合并成"两只宠物的体重"。
2. 主题名称要具体到问题本身，比如"体重管理"、"疫苗接种计划"，而不是笼统的"健康咨询"。
3. 摘要要保留关键客观事实：宠物名字、品种、年龄、体重数字、症状、用药、时间。
4. 关键词给 2 到 5 个，只写话题本身的词，**不要写宠物名字或品种**——
   同一只宠物的不同话题若都带上它的名字，就会被误判成同一个话题而合并到一起。
5. turns 填该主题包含的对话序号。
6. 只输出 JSON，不要任何解释文字。

输出格式：
{"summaries": [{"theme": "主题名", "content": "主题摘要", "keywords": ["关键词"], "turns": [1, 2]}]}"""

SUMMARY_USER = """请对下面的对话做主题摘要。

{dialogue}"""


PROFILE_SYSTEM = """你是宠物健康助手的用户画像分析员。根据对话内容，提取本次新观察到的用户画像信息。

重要：只输出这一批对话里**新出现或有变化**的信息。历史画像由系统自行合并保存，你不需要重复已知内容，也不要试图总结用户的全部情况。

字段说明：
- communication: 沟通偏好，如专业术语接受度、回答详略偏好、语气偏好
- concerns: 用户反复关注的方面，字符串数组
- petFacts: 宠物的客观事实，按宠物名字分组，如品种、年龄、体重、过敏源、慢性病
- healthWatch: 需要持续留意的健康问题，字符串数组

规则：
1. 只写对话中明确提到的内容，不推测、不脑补。
2. 某个字段这次没有新信息就不要出现在输出里，不要填"无"或空字符串。
3. 只输出 JSON，不要任何解释文字。

输出格式：
{"communication": {}, "concerns": [], "petFacts": {}, "healthWatch": []}"""

PROFILE_USER = """请从下面的对话中提取用户画像的新增信息。

{dialogue}"""


KNOWLEDGE_SYSTEM = """你是宠物健康助手的长期记忆抽取员。从对话中抽取值得长期记住的知识条目。

什么值得记：宠物的客观属性与病史、用户明确表达的偏好与禁忌、已确定的用药或饮食方案、重要的时间节点。
什么不值得记：一次性的寒暄、助手给出的通用科普、可以随时重新查到的常识、模糊的猜测。

规则：
1. 每条写成一句可独立理解的话，包含主语，比如"布偶猫咪咪对鸡肉过敏"而不是"对鸡肉过敏"。
2. 没有值得记的内容就返回空数组，不要为了凑数而写。
3. 最多 5 条。
4. 只输出 JSON，不要任何解释文字。

输出格式：
{"facts": ["知识条目"]}"""

KNOWLEDGE_USER = """请从下面的对话中抽取值得长期记住的知识条目。

{dialogue}"""


def format_dialogue(turns: Sequence[Dict[str, Any]], numbered: bool = True) -> str:
    """把对话轮次拼成提示词里的文本块。

    序号是给主题归类用的：模型要靠它在 turns 字段里指明每个主题包含哪几轮。
    """
    lines: List[str] = []
    for index, turn in enumerate(turns, start=1):
        prefix = f"[{index}] " if numbered else ""
        lines.append(f"{prefix}用户: {turn.get('user_input', '')}")
        lines.append(f"{prefix}助手: {turn.get('agent_response', '')}")
    return "\n".join(lines)


def summary_messages(turns: Sequence[Dict[str, Any]]) -> List[Dict[str, str]]:
    return [
        {"role": "system", "content": SUMMARY_SYSTEM},
        {"role": "user", "content": SUMMARY_USER.format(dialogue=format_dialogue(turns))},
    ]


def profile_messages(turns: Sequence[Dict[str, Any]]) -> List[Dict[str, str]]:
    return [
        {"role": "system", "content": PROFILE_SYSTEM},
        {"role": "user", "content": PROFILE_USER.format(dialogue=format_dialogue(turns, numbered=False))},
    ]


def knowledge_messages(turns: Sequence[Dict[str, Any]]) -> List[Dict[str, str]]:
    return [
        {"role": "system", "content": KNOWLEDGE_SYSTEM},
        {"role": "user", "content": KNOWLEDGE_USER.format(dialogue=format_dialogue(turns, numbered=False))},
    ]
