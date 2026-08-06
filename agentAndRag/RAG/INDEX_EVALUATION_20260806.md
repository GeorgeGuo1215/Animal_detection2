# 兽医知识库索引重建与检索验证报告（2026-08-06）

## 结论

本次没有在旧碎片化索引上继续追加，而是使用 `multilingual-e5-small` 完成全量重建并切换运行时目录。最终索引包含 79 份唯一正文、110,964 个语义块和 54 个分类（43 个非空）；逐书原文片段 top-5 召回 79/79，通过率 100%，分类缺块与串库均为 0。

旧索引保留在 `RAG/data/*_legacy_20260806`，新旧索引文件均由 `.gitignore` 排除。分类标准 Excel、taxonomy、建库/验证脚本和本报告可纳入 Git。

## 为什么采用全量重建

| 指标 | 旧索引 | semantic-v2 |
|---|---:|---:|
| 主索引块数 | 166,990 | 110,964 |
| 块长中位数 | 92 | 345 |
| 块长 P90 | 212 | 378 |
| 小于 100 units 的块 | 91,215 | 195 |

旧切分器对每个 OCR 段落单独应用最短长度，导致页眉、短段和表格行大量碎片化；中文又因按空白计数而被严重低估。新策略按“英文单词/中文汉字”统一计数，聚合相邻短段，超长段按句子切分，默认目标 380、重叠 60，并保留书尾短段。

## 输入资料核对

| 项目 | 数量 | 处理 |
|---|---:|---|
| OCR 输出顶层目录 | 88 | 物理目录，不等于唯一书籍数 |
| 分类表唯一书号 | 85 | 以分类 2.0 Excel 为准 |
| 可解析书号 | 84 | `084` 正在单独执行递归 OCR，未使用半成品 |
| 主动排除 | 2 | `078` 多语混合；`005` 正文与书名/分类错配 |
| 完全重复别名 | 3 | `046→045`、`063→062`、`015→012` |
| 最终唯一正文 | 79 | 实际进入索引并逐书验证 |

88 个输出目录中未被分类表作为独立书籍使用的 4 个目录为：`081`（与 `040` 同书）以及三个 `64*` 目录（同一本 *Questions & Answers in Anatomy & Physiology for Veterinary Nurses* 的重复/近重复 OCR，分类表无书号）。

新增资料语言粗检显示，除 `078` 外其余新增 MMD 均以英文为主；`082` 包含中兽医药名相关汉字，属于有效专业内容。`078` 的西语特征词约为英语特征词的 19.3%，并含其他语种段落，因此在完成全量医学翻译和术语质检前不入库。

`005` 的正文标题实际为 *Advances in Reproduction*，与 `008` 内容 SHA1 完全相同，却在表中归为 `equine.complications`；为防止类别污染，保留正确的 `008` 并排除 `005`。

## 构建结果

| 项目 | 结果 |
|---|---:|
| GPU | NVIDIA GeForce RTX 3080 Ti Laptop GPU |
| embedding | `intfloat/multilingual-e5-small`，384 维 |
| GPU 向量化与落盘耗时 | 1,124.7 秒 |
| 主索引行数 | 110,964 |
| 分类索引行数合计 | 111,244 |
| 分类数 / 非空分类数 | 54 / 43 |
| 主索引大小 | 0.456 GiB |
| 分类索引大小 | 0.457 GiB |

分类索引比主索引多 280 行，是因为 `027` 按分类表同时属于 `basic.anatomy` 和 `diagnostics.imaging`，属于预期的一书多类。

新增类别已映射为稳定英文 ID，并接入专家通配符：`basic.histology`、`clinical.reference`、`pharmacy.clinical_therapeutics`、`infectious.general`、`nutrition.general`、`integrative.tcm_support`、`individual.genetics_breed`、`guidelines.general`。临床专家同时补充 `basic.*`、`infectious.*`、`equine.*`、`individual.*`、`guidelines.*` 等范围。

## 逐书原文片段验证

验证方法：每本书选择中部 chunk 的开头原文片段，在其所属分类索引中执行 E5 dense top-5 检索；同时遍历所有分类元数据检查书籍成员关系。完整片段、分数和 top-5 结果保存在本地忽略文件 `RAG/data/retrieval_validation_v2.json`。

- top-5：79/79；
- top-1：78/79；
- `017` 为 top-2，top-1 是同主题的 `016 Equine Acute Abdomen`；
- 分类缺块：0；
- 类别串库：0。

| 书号 | 分类 | 排名 | 书号 | 分类 | 排名 |
|---|---|---:|---|---|---:|
| 004 | `clinical_skills.nursing` | 1 | 006 | `equine.sports_medicine` | 1 |
| 007 | `equine.oncology` | 1 | 008 | `equine.reproduction` | 1 |
| 009 | `equine.cardiology` | 1 | 010 | `equine.sports_medicine` | 1 |
| 011 | `equine.colic` | 1 | 012 | `equine.colic` | 1 |
| 013 | `equine.colic` | 1 | 014 | `equine.colic` | 1 |
| 016 | `equine.colic` | 1 | 017 | `equine.colic` | 2 |
| 018 | `equine.infectious` | 1 | 019 | `zoonosis.toxoplasmosis` | 1 |
| 020 | `equine.neurology` | 1 | 021 | `equine.neurology` | 1 |
| 022 | `equine.nutrition` | 1 | 023 | `equine.large_animal_internal` | 1 |
| 024 | `clinical.surgery` | 1 | 025 | `anesthesia.default` | 1 |
| 026 | `pharmacy.applied_pharmacology` | 1 | 027 | `basic.anatomy` | 1 |
| 028 | `exotic.default` | 1 | 029 | `behavior.dog_cat_problems` | 1 |
| 030 | `diagnostics.clinical_pathology` | 1 | 031 | `clinical.cardiology` | 1 |
| 032 | `clinical_skills.nursing` | 1 | 033 | `clinical.internal_medicine` | 1 |
| 034 | `clinical.neurology` | 1 | 035 | `exotic.default` | 1 |
| 036 | `basic.anatomy` | 1 | 037 | `clinical.internal_medicine` | 1 |
| 038 | `exotic.default` | 1 | 039 | `basic.anatomy` | 1 |
| 040 | `integrative.tcm_support` | 1 | 041 | `exotic.default` | 1 |
| 042 | `diagnostics.laboratory` | 1 | 043 | `diagnostics.laboratory` | 1 |
| 044 | `diagnostics.imaging` | 1 | 045 | `clinical.cardiology` | 1 |
| 047 | `diagnostics.differential` | 1 | 048 | `basic.anatomy` | 1 |
| 049 | `pharmacy.papich` | 1 | 050 | `clinical.ophthalmology` | 1 |
| 051 | `clinical_skills.techniques` | 1 | 052 | `clinical.emergency_critical` | 1 |
| 053 | `clinical.dermatology` | 1 | 054 | `diagnostics.imaging` | 1 |
| 055 | `clinical.internal_medicine` | 1 | 056 | `clinical.surgery` | 1 |
| 057 | `clinical_skills.nursing` | 1 | 058 | `clinical.internal_medicine` | 1 |
| 059 | `diagnostics.imaging` | 1 | 060 | `diagnostics.imaging` | 1 |
| 061 | `clinical.dermatology` | 1 | 062 | `immunology.default` | 1 |
| 064 | `basic.terminology` | 1 | 065 | `reproduction.default` | 1 |
| 066 | `clinical.surgery` | 1 | 067 | `behavior.feline_welfare` | 1 |
| 068 | `basic.pathology` | 1 | 069 | `basic.pathology` | 1 |
| 070 | `pharmacy.clinical_therapeutics` | 1 | 071 | `clinical.oncology` | 1 |
| 072 | `diagnostics.laboratory` | 1 | 073 | `basic.histology` | 1 |
| 074 | `infectious.general` | 1 | 075 | `infectious.general` | 1 |
| 076 | `nutrition.general` | 1 | 077 | `nutrition.general` | 1 |
| 079 | `clinical.reference` | 1 | 080 | `individual.genetics_breed` | 1 |
| 082 | `integrative.tcm_support` | 1 | 083 | `clinical.reference` | 1 |
| 085 | `guidelines.general` | 1 | 086 | `guidelines.general` | 1 |
| 087 | `guidelines.general` | 1 | 088 | `guidelines.general` | 1 |
| 089 | `guidelines.general` | 1 |  |  |  |

## Agent 运行时链路抽查

通过真实 `rag_search_tool → taxonomy → 分类索引` 调用，而非绕过 Agent 直接查数组：

| 查询主题 | 分类 | Top-1 |
|---|---|---|
| Jubb pathology volume 2 | `basic.pathology` | `069`（top-2 为 `068`） |
| Blackwell five-minute consult | `clinical.reference` | `083` |
| small animal clinical nutrition | `nutrition.general` | `076` |
| small animal clinical pharmacology | `pharmacy.clinical_therapeutics` | `070` |
| ACVIM mitral valve consensus | `guidelines.general` | `088` |

运行时相对路径解析结果正常，临床、营养、药学、指南和麻醉分类目录均存在。

## 回归结果与待办

- `python -m pytest RAG/tests -q`：13 passed；
- `pytest agent_api/tests/rag/test_category_search.py agent_api/tests/moe/test_expert_rag_categories.py -q`：9 passed；
- OCR 递归发现测试覆盖 `batch/084/volume-1/guide.pdf`；
- Python compileall：通过。

待 `084` OCR 完成后，使用 README 中同一临时目录重建与验证命令重新生成索引；若要纳入三个 `64*` 生理学问答目录，应先在分类表新增唯一书号并指定唯一规范源，不能直接把三个目录全部追加。

`084` 的源目录包含多层子目录，OCR 发现逻辑已改为递归扫描；首次递归发现的 47 个 PDF 中有 11 个 SHA1 完全相同的 `download.pdf`，内容去重后为 37 份唯一 PDF。由于 PDF 列表发生变化，旧的分页序号不能续用，需以 `--force` 从 084 第一页重建。
