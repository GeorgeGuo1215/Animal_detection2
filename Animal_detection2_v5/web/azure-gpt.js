/**
 * Azure OpenAI GPT集成模块
 * 用于分析心率和呼吸率数据，生成诊断报告
 */

class AzureGPTAnalyzer {
    constructor() {
        this.config = {
            endpoint: '',
            apiKey: '',
            deploymentName: '',
            apiVersion: '2024-02-15-preview'
        };
        
        this.isConfigured = false;
        this.ragDatabase = new Map(); // 简单的RAG知识库
        this.customPrompts = new Map(); // 自定义prompt存储
        
        this.initializeDefaultPrompts();
        this.initializeRAGDatabase();
    }

    /**
     * 配置Azure OpenAI连接
     */
    configure(endpoint, apiKey, deploymentName) {
        this.config.endpoint = endpoint.replace(/\/$/, ''); // 移除末尾斜杠
        this.config.apiKey = apiKey;
        this.config.deploymentName = deploymentName;
        this.isConfigured = true;
        
        console.log('Azure OpenAI配置完成:', {
            endpoint: this.config.endpoint,
            deploymentName: this.config.deploymentName,
            hasApiKey: !!this.config.apiKey
        });
    }

    /**
     * 初始化默认prompt模板
     */
    initializeDefaultPrompts() {
        this.customPrompts.set('basic_analysis', {
            name: '基础生理参数分析',
            template: `你是一位专业的心血管医生。请分析以下生理参数数据并提供诊断建议：

心率数据：{heartRateData}
呼吸频率数据：{respiratoryData}
测量时间：{measurementTime}
患者信息：{patientInfo}

请从以下几个方面进行分析：
1. 心率变异性分析
2. 呼吸模式评估
3. 心肺功能协调性
4. 异常指标识别
5. 健康风险评估
6. 建议和注意事项

请提供专业、详细的医学分析报告。`,
            description: '基础的心率和呼吸频率分析'
        });

        this.customPrompts.set('detailed_medical', {
            name: '详细医学诊断',
            template: `作为心血管专科医生，请对以下毫米波雷达检测的生理数据进行详细医学分析：

## 检测数据
- 平均心率：{avgHeartRate} bpm
- 心率范围：{heartRateRange}
- 心率变异性：{heartRateVariability}
- 平均呼吸频率：{avgRespiratoryRate} bpm
- 呼吸模式：{respiratoryPattern}
- 检测时长：{duration}
- 数据质量：{dataQuality}

## 参考知识
{ragContext}

请提供包含以下内容的诊断报告：
1. **生理参数评估**：对比正常范围，评估各项指标
2. **病理学分析**：识别可能的异常模式
3. **风险分层**：评估心血管风险等级
4. **临床建议**：提供具体的医疗建议
5. **随访计划**：建议后续监测方案
6. **生活方式指导**：日常保健建议

报告应专业、准确、易懂。`,
            description: '详细的医学诊断报告，包含风险评估和建议'
        });

        this.customPrompts.set('trend_analysis', {
            name: '趋势分析报告',
            template: `请分析以下时间序列的心率和呼吸数据趋势：

## 时间序列数据
心率时间序列：{heartRateTimeSeries}
呼吸频率时间序列：{respiratoryTimeSeries}
时间轴：{timeAxis}

## 分析要求
1. 识别数据中的周期性模式
2. 检测异常波动和突变点
3. 评估整体趋势（上升/下降/稳定）
4. 分析心率变异性特征
5. 评估心肺同步性
6. 预测潜在健康风险

请提供趋势分析报告，重点关注时间变化模式和临床意义。`,
            description: '基于时间序列的趋势分析'
        });
    }

    /**
     * 初始化RAG知识库
     */
    initializeRAGDatabase() {
        // 心率相关知识
        this.ragDatabase.set('heart_rate_normal', {
            content: `正常成年人静息心率范围：
- 一般成年人：60-100 bpm
- 运动员：40-60 bpm
- 老年人：60-80 bpm
- 儿童：80-120 bpm

心率异常分类：
- 心动过缓：<60 bpm
- 心动过速：>100 bpm
- 严重心动过缓：<40 bpm
- 严重心动过速：>150 bpm`,
            keywords: ['心率', '正常范围', '心动过缓', '心动过速']
        });

        this.ragDatabase.set('respiratory_normal', {
            content: `正常呼吸频率范围：
- 成年人：12-20 次/分钟
- 儿童：20-30 次/分钟
- 婴儿：30-40 次/分钟

呼吸异常分类：
- 呼吸过缓：<12 次/分钟
- 呼吸过速：>20 次/分钟
- 严重呼吸困难：>30 次/分钟

呼吸模式异常：
- 不规则呼吸
- 呼吸暂停
- 浅快呼吸`,
            keywords: ['呼吸频率', '正常范围', '呼吸过缓', '呼吸过速']
        });

        this.ragDatabase.set('heart_rate_variability', {
            content: `心率变异性(HRV)临床意义：
- 高HRV：通常表示良好的自主神经功能
- 低HRV：可能提示：
  * 自主神经功能失调
  * 心血管疾病风险增加
  * 压力水平过高
  * 疲劳状态

HRV评估指标：
- RMSSD：连续RR间期差值的均方根
- SDNN：RR间期标准差
- pNN50：相邻RR间期差值>50ms的百分比`,
            keywords: ['心率变异性', 'HRV', '自主神经', '心血管风险']
        });

        this.ragDatabase.set('cardiopulmonary_coupling', {
            content: `心肺耦合分析：
正常情况下，心率和呼吸存在生理性耦合：
- 呼吸性窦性心律不齐：吸气时心率加快，呼气时心率减慢
- 正常耦合比例：心率/呼吸频率 ≈ 4:1

异常耦合模式：
- 耦合消失：可能提示自主神经功能异常
- 过度耦合：可能提示心功能不全
- 反向耦合：罕见，需进一步检查`,
            keywords: ['心肺耦合', '窦性心律不齐', '自主神经', '心功能']
        });

        this.ragDatabase.set('risk_assessment', {
            content: `心血管风险评估标准：
低风险：
- 心率60-80 bpm，规律
- 呼吸12-16次/分钟，规律
- 良好的心率变异性

中等风险：
- 轻度心动过速(100-120 bpm)或心动过缓(50-60 bpm)
- 轻度呼吸异常
- 心率变异性轻度降低

高风险：
- 严重心律异常
- 明显呼吸异常
- 心率变异性显著降低
- 心肺耦合异常`,
            keywords: ['风险评估', '心血管风险', '心律异常', '呼吸异常']
        });
    }

    /**
     * 添加自定义prompt
     */
    addCustomPrompt(id, name, template, description = '') {
        this.customPrompts.set(id, {
            name,
            template,
            description,
            createdAt: new Date().toISOString()
        });
        
        console.log(`添加自定义prompt: ${name}`);
    }

    /**
     * 获取所有可用的prompt
     */
    getAvailablePrompts() {
        const prompts = [];
        for (const [id, prompt] of this.customPrompts) {
            prompts.push({
                id,
                name: prompt.name,
                description: prompt.description
            });
        }
        return prompts;
    }

    /**
     * 添加RAG知识条目
     */
    addRAGEntry(id, content, keywords = []) {
        this.ragDatabase.set(id, {
            content,
            keywords,
            addedAt: new Date().toISOString()
        });
        
        console.log(`添加RAG知识条目: ${id}`);
    }

    /**
     * 检索相关RAG内容
     */
    retrieveRAGContext(query, maxResults = 3) {
        const queryKeywords = query.toLowerCase().split(/\s+/);
        const relevantEntries = [];
        
        for (const [id, entry] of this.ragDatabase) {
            let relevanceScore = 0;
            
            // 计算关键词匹配度
            for (const keyword of entry.keywords) {
                for (const queryKeyword of queryKeywords) {
                    if (keyword.toLowerCase().includes(queryKeyword) || 
                        queryKeyword.includes(keyword.toLowerCase())) {
                        relevanceScore += 1;
                    }
                }
            }
            
            // 检查内容匹配度
            const contentLower = entry.content.toLowerCase();
            for (const queryKeyword of queryKeywords) {
                if (contentLower.includes(queryKeyword)) {
                    relevanceScore += 0.5;
                }
            }
            
            if (relevanceScore > 0) {
                relevantEntries.push({
                    id,
                    content: entry.content,
                    score: relevanceScore
                });
            }
        }
        
        // 按相关性排序并返回前N个结果
        return relevantEntries
            .sort((a, b) => b.score - a.score)
            .slice(0, maxResults)
            .map(entry => entry.content)
            .join('\n\n');
    }

    /**
     * 准备分析数据
     */
    prepareAnalysisData(processedResults) {
        if (!processedResults || processedResults.length === 0) {
            throw new Error('没有可分析的数据');
        }
        
        const successResults = processedResults.filter(r => r.status === 'success');
        if (successResults.length === 0) {
            throw new Error('没有成功处理的数据');
        }
        
        // 计算统计数据
        const heartRates = successResults.map(r => r.heartRate).filter(hr => hr > 0);
        const respiratoryRates = successResults.map(r => r.respiratoryRate).filter(rr => rr > 0);
        
        const avgHeartRate = heartRates.length > 0 ? 
            (heartRates.reduce((a, b) => a + b, 0) / heartRates.length).toFixed(1) : 0;
        const avgRespiratoryRate = respiratoryRates.length > 0 ? 
            (respiratoryRates.reduce((a, b) => a + b, 0) / respiratoryRates.length).toFixed(1) : 0;
        
        const heartRateRange = heartRates.length > 0 ? 
            `${Math.min(...heartRates)}-${Math.max(...heartRates)} bpm` : 'N/A';
        
        // 计算心率变异性（简化版本）
        const heartRateVariability = heartRates.length > 1 ? 
            Math.sqrt(heartRates.reduce((sum, hr, i, arr) => {
                if (i === 0) return sum;
                return sum + Math.pow(hr - arr[i-1], 2);
            }, 0) / (heartRates.length - 1)).toFixed(2) : 'N/A';
        
        // 准备时间序列数据
        const firstResult = successResults[0];
        const heartRateTimeSeries = firstResult.heartRateTimeSeries || heartRates;
        const respiratoryTimeSeries = firstResult.respiratoryRateTimeSeries || respiratoryRates;
        const timeAxis = firstResult.timeAxis || heartRates.map((_, i) => i);
        
        return {
            avgHeartRate,
            avgRespiratoryRate,
            heartRateRange,
            heartRateVariability,
            heartRateData: heartRates.join(', '),
            respiratoryData: respiratoryRates.join(', '),
            heartRateTimeSeries: heartRateTimeSeries.slice(0, 20).join(', '), // 限制长度
            respiratoryTimeSeries: respiratoryTimeSeries.slice(0, 20).join(', '),
            timeAxis: timeAxis.slice(0, 20).join(', '),
            duration: `${successResults.length} 个数据段`,
            dataQuality: this.assessDataQuality(successResults),
            measurementTime: new Date().toLocaleString('zh-CN'),
            patientInfo: '未提供', // 可以后续扩展
            respiratoryPattern: this.analyzeRespiratoryPattern(respiratoryRates)
        };
    }

    /**
     * 评估数据质量
     */
    assessDataQuality(results) {
        const successRate = results.filter(r => r.status === 'success').length / results.length;
        if (successRate >= 0.9) return '优秀';
        if (successRate >= 0.7) return '良好';
        if (successRate >= 0.5) return '一般';
        return '较差';
    }

    /**
     * 分析呼吸模式
     */
    analyzeRespiratoryPattern(respiratoryRates) {
        if (respiratoryRates.length === 0) return '无数据';
        
        const avg = respiratoryRates.reduce((a, b) => a + b, 0) / respiratoryRates.length;
        const variance = respiratoryRates.reduce((sum, rate) => sum + Math.pow(rate - avg, 2), 0) / respiratoryRates.length;
        const stdDev = Math.sqrt(variance);
        
        if (stdDev < 2) return '规律呼吸';
        if (stdDev < 4) return '轻度不规律';
        return '明显不规律';
    }

    /**
     * 生成诊断报告
     */
    async generateDiagnosticReport(processedResults, promptId = 'basic_analysis', customPrompt = null) {
        if (!this.isConfigured) {
            throw new Error('Azure OpenAI未配置，请先调用configure()方法');
        }
        
        try {
            // 准备分析数据
            const analysisData = this.prepareAnalysisData(processedResults);
            
            // 获取prompt模板
            let promptTemplate;
            if (customPrompt) {
                promptTemplate = customPrompt;
            } else if (this.customPrompts.has(promptId)) {
                promptTemplate = this.customPrompts.get(promptId).template;
            } else {
                throw new Error(`未找到prompt模板: ${promptId}`);
            }
            
            // 检索RAG上下文
            const ragQuery = `心率 ${analysisData.avgHeartRate} 呼吸频率 ${analysisData.avgRespiratoryRate}`;
            const ragContext = this.retrieveRAGContext(ragQuery);
            analysisData.ragContext = ragContext;
            
            // 替换模板变量
            const finalPrompt = this.replaceTemplateVariables(promptTemplate, analysisData);
            
            console.log('发送到Azure OpenAI的prompt长度:', finalPrompt.length);
            
            // 调用Azure OpenAI API
            const response = await this.callAzureOpenAI(finalPrompt);
            
            return {
                success: true,
                report: response,
                analysisData,
                promptUsed: promptId,
                ragContext,
                timestamp: new Date().toISOString()
            };
            
        } catch (error) {
            console.error('生成诊断报告失败:', error);
            return {
                success: false,
                error: error.message,
                timestamp: new Date().toISOString()
            };
        }
    }

    /**
     * 替换模板变量
     */
    replaceTemplateVariables(template, data) {
        let result = template;
        
        for (const [key, value] of Object.entries(data)) {
            const placeholder = `{${key}}`;
            result = result.replace(new RegExp(placeholder, 'g'), value);
        }
        
        return result;
    }

    /**
     * 调用Azure OpenAI API
     */
    async callAzureOpenAI(prompt) {
        const url = `${this.config.endpoint}/openai/deployments/${this.config.deploymentName}/chat/completions?api-version=${this.config.apiVersion}`;
        
        const requestBody = {
            messages: [
                {
                    role: "system",
                    content: "你是一位专业的心血管医生，具有丰富的临床经验。请基于提供的生理参数数据进行专业分析。"
                },
                {
                    role: "user",
                    content: prompt
                }
            ],
            max_tokens: 2000,
            temperature: 0.3,
            top_p: 0.9,
            frequency_penalty: 0,
            presence_penalty: 0
        };
        
        const response = await fetch(url, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
                'api-key': this.config.apiKey
            },
            body: JSON.stringify(requestBody)
        });
        
        if (!response.ok) {
            const errorText = await response.text();
            throw new Error(`Azure OpenAI API调用失败: ${response.status} - ${errorText}`);
        }
        
        const data = await response.json();
        
        if (data.choices && data.choices.length > 0) {
            return data.choices[0].message.content;
        } else {
            throw new Error('Azure OpenAI返回了空响应');
        }
    }

    /**
     * 导出RAG知识库
     */
    exportRAGDatabase() {
        const ragData = {};
        for (const [id, entry] of this.ragDatabase) {
            ragData[id] = entry;
        }
        return ragData;
    }

    /**
     * 导入RAG知识库
     */
    importRAGDatabase(ragData) {
        for (const [id, entry] of Object.entries(ragData)) {
            this.ragDatabase.set(id, entry);
        }
        console.log(`导入了 ${Object.keys(ragData).length} 条RAG知识`);
    }

    /**
     * 导出自定义prompt
     */
    exportCustomPrompts() {
        const promptData = {};
        for (const [id, prompt] of this.customPrompts) {
            promptData[id] = prompt;
        }
        return promptData;
    }

    /**
     * 导入自定义prompt
     */
    importCustomPrompts(promptData) {
        for (const [id, prompt] of Object.entries(promptData)) {
            this.customPrompts.set(id, prompt);
        }
        console.log(`导入了 ${Object.keys(promptData).length} 个自定义prompt`);
    }
}

// 导出供其他模块使用
if (typeof module !== 'undefined' && module.exports) {
    module.exports = AzureGPTAnalyzer;
} else {
    window.AzureGPTAnalyzer = AzureGPTAnalyzer;
}

