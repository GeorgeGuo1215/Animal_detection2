"""Fact-state instructions injected into every MoE reasoning stage."""

HISTORY_RULES = (
    "以下是后端保存的既往上下文，状态标签不可被后续模型改写：\n"
    "- user_report：用户原始陈述。症状和病史属于用户报告；只有用户明确说已由检查或兽医确诊时，"
    "才能视为 confirmed_fact。\n"
    "- assistant_inference：先前模型输出，只能证明模型曾提出该内容，不能证明患者存在其中的诊断、"
    "分期、检查结果或用药事实。\n"
    "- expert_inference：先前专家 Subagent 的推断、工具决策和摘要，不是新增患者事实；检索资料只证明"
    "一般医学知识，不自动证明其适用于当前患者。\n"
    "- user_memory / cross_session_memory：同一用户名绑定的跨会话用户记忆。其中用户陈述部分可直接"
    "用于身份确认与健康随访（如宠物名、物种/品种、年龄、既往主诉）；其中助手回复仍属推断，不得"
    "单独升级为确诊。新开会话、本轮 conversation 为空时，仍必须优先使用该记忆，不得声称“没有既往信息”。\n"
    "若后续用户没有提供新的确认依据，必须保持原有不确定性；不得把可能、疑似或待排查升级为既往史、"
    "确诊、确定分期或确定用药指征。"
)

FACT_STATE_HISTORY_PREFIX = "历史事实状态提示（FACT_STATE_HISTORY）：所有状态标签必须跨轮保持。\n"
