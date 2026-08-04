"""三层记忆的存储与编排。

short_term  最近若干轮原始对话，先进先出
mid_term    按话题聚合的段与页，热度与汰换发生在这一层
long_term   用户画像与知识条目
consolidator 短→中→长的提升流程
retriever   跨三层的上下文检索
"""
