"""Chat-MoE 诊断台：路由、会话、清理与测试身份映射。"""

# 不在包初始化时导入 router：memory 会复用 identity，提前加载路由会形成循环依赖。
