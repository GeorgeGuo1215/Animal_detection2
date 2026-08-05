"""
一个尽量“简单、可离线、Windows 友好”的 RAG（检索增强生成）实现。

本包只负责：
- 从 .mmd 文档中提取文本、分块
- 生成 embedding
- 建立/持久化向量索引
- 执行相似度检索并返回引用片段
"""

from .query_rewrite import NoRewrite, TemplateRewriter
from .retrieval import MultiRouteRetriever, build_default_multiroute



