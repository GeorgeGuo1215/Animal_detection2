import { memo } from 'react'
import ReactMarkdown from 'react-markdown'
import rehypeSanitize from 'rehype-sanitize'
import { ChevronDown, Stethoscope } from 'lucide-react'
import { agentProcessSummary, traceNodeTitle } from './agentProcess'
import type { ExpertTrace, TraceNode, Message } from './types'

const traceStatusText: Record<string, string> = { pending: '等待执行', running: '执行中', completed: '已完成', degraded: '证据不足', failed: '失败', cancelled: '已取消' }
const sufficiencyText: Record<string, string> = { supported: '证据充分', partial: '部分支持', unsupported: '证据不足', unknown: '尚未核实' }

function TraceTimeline({ nodes }: { nodes: TraceNode[] }) {
  const visible = nodes.filter(node => node.node_type !== 'expert')
  if (!visible.length) return null
  return <div className="trace-timeline" aria-label="会诊任务轨迹">
    {visible.map(node => {
      const detail = node.details || {}
      const title = traceNodeTitle(node)
      const body = node.node_type === 'decision'
        ? `输出结构 ${detail.output_variant || 'default'}${detail.emergency ? ' · 已标记急症风险' : ''}`
        : node.node_type === 'goal' ? detail.goal
        : node.node_type === 'query' ? detail.query
        : node.node_type === 'review' ? (detail.verdict ? `复核结果：${detail.verdict}` : '检查事实边界、用药安全与证据引用')
        : detail.message || (node.status === 'completed' ? '最终答复已完成' : '结合专家意见与证据生成答复')
      const sufficiency = detail.sufficiency?.status
      return <div className={`trace-node ${node.node_type} ${node.status}`} key={node.node_id}>
        <span className="trace-node-dot" />
        <div><div className="trace-node-head"><strong>{title}</strong><small>{traceStatusText[node.status] || node.status}</small></div>
          {body && <p>{body}</p>}
          {node.node_type === 'query' && <div className="trace-node-meta">
            {detail.scope === 'expanded' && <span>扩大分类</span>}
            {!!detail.latency_ms && <span>{(detail.latency_ms / 1000).toFixed(1)} 秒</span>}
            {sufficiency && <span className={`sufficiency ${sufficiency}`}>{sufficiencyText[sufficiency] || sufficiency}</span>}
            {detail.result?.hits !== undefined && <span>命中 {detail.result.hits} 条</span>}
            {detail.result?.results !== undefined && <span>结果 {detail.result.results} 条</span>}
          </div>}
          {detail.sufficiency?.reason && <small className="trace-reason">{detail.sufficiency.reason}</small>}
          {detail.error && <small className="trace-error">{detail.error}</small>}
        </div>
      </div>
    })}
  </div>
}

function AgentProcessFlow({ nodes, phase, active, status }: { nodes: TraceNode[]; phase: string; active: boolean; status: string }) {
  const summary = status === 'cancelled' ? '生成已停止，内容可能不完整' : status === 'failed' ? '生成失败，内容可能不完整' : agentProcessSummary(nodes, phase, active)
  return <details className={`agent-process-flow ${active ? 'active' : 'complete'}`}>
    <summary>
      <span className="agent-process-mark" aria-hidden="true" />
      <span><strong>Agent 处理流程</strong><small aria-live="polite">{summary}</small></span>
      <span className="agent-process-state">{active ? '处理中' : status === 'cancelled' ? '已停止' : status === 'failed' ? '失败' : '已完成'}</span>
      <ChevronDown />
    </summary>
    <div className="agent-process-body">
      <TraceTimeline nodes={nodes} />
      {!nodes.some(node => node.node_type !== 'expert') && <p className="agent-process-waiting">{summary}，详细步骤将在执行后显示。</p>}
      <p className="expert-privacy">仅展示公开任务阶段与检索结果，不展示系统提示词或模型内部推理。</p>
    </div>
  </details>
}

function ExpertConsultation({ experts, traceNodes = [], phase = '', active = false, status = 'complete' }: { experts: ExpertTrace[]; traceNodes?: TraceNode[]; phase?: string; active?: boolean; status?: string }) {
  const defaultOpen = localStorage.getItem('petmind-default-expand-experts') !== 'false'
  if (!experts.length && !traceNodes.length && !active) return null
  return <section className="expert-consultation" aria-label="Agent 与专家会诊过程">
    <AgentProcessFlow nodes={traceNodes} phase={phase} active={active} status={status} />
    {experts.length > 0 && <details className="expert-panel">
      <summary className="expert-consultation-heading"><Stethoscope /><div><strong>专家会诊</strong><small>{active ? '专家正在执行会诊任务' : `${experts.length} 位专家已提交会诊意见`}</small></div><span>{active ? '会诊中' : '已完成'}</span><ChevronDown /></summary>
      <div className="expert-panel-body">{experts.map(expert => <details className="expert-thread" open={defaultOpen || undefined} key={expert.expert}>
      <summary><span className={`expert-dot ${expert.status}`} /><span><strong>{expert.name}</strong><small>{expert.status === 'completed' ? '已提交结构化意见' : '正在执行任务'}</small></span><ChevronDown /></summary>
      <div className="expert-thread-body">
        <div className="expert-step"><span>任务</span><p>{expert.task || '根据统一任务策略分析当前病例'}</p></div>
        {(expert.tools || []).map((tool, index) => <div className="expert-step" key={`${tool.tool_name}-${index}`}><span>工具</span><div><strong>{tool.tool_name}</strong><small>{tool.ok ? '调用完成' : '调用失败'} · {(tool.latency_ms / 1000).toFixed(1)} 秒{tool.wave && tool.wave > 1 ? ` · 第 ${tool.wave} 波` : ''}{tool.scope === 'expanded' ? ' · 扩大分类' : ''}</small>{tool.query && <p className="expert-query">{tool.query}</p>}{tool.sufficiency && <small className={`sufficiency ${tool.sufficiency.status}`}>{sufficiencyText[tool.sufficiency.status] || tool.sufficiency.status} · {tool.sufficiency.reason}</small>}{tool.result?.hits !== undefined && <p>知识库命中 {tool.result.hits} 条{tool.result.sources?.length ? ` · ${tool.result.sources.join('、')}` : ''}</p>}{tool.result?.results !== undefined && <p>网络结果 {tool.result.results} 条{tool.result.titles?.length ? ` · ${tool.result.titles.join('、')}` : ''}</p>}{tool.error && <p className="expert-step-error">{tool.error}</p>}</div></div>)}
        {expert.opinion && <div className="expert-step"><span>意见</span><div><p>{expert.opinion.conclusion}</p>{expert.opinion.evidence.length > 0 && <ul>{expert.opinion.evidence.map(item => <li key={item}>{item}</li>)}</ul>}{expert.opinion.risks.length > 0 && <small>风险边界：{expert.opinion.risks.join('；')}</small>}</div></div>}
        <p className="expert-privacy">仅展示任务结果，不展示系统提示词或模型内部推理。</p>
      </div>
      </details>)}</div>
    </details>}
  </section>
}


export const AssistantAnswer = memo(function AssistantAnswer({ message, phase }: { message: Message; phase: string }) {
  return <><div className="message-label"><Stethoscope />PetMind 会诊意见</div><ExpertConsultation experts={message.expert_consultations || []} traceNodes={message.trace_nodes || []} phase={message.status === 'streaming' ? phase : ''} active={message.status === 'streaming'} status={message.status} />{message.content && <div className={`message-body ${message.status === 'streaming' ? 'streaming-markdown is-streaming' : ''}`}><ReactMarkdown rehypePlugins={[rehypeSanitize]}>{message.content}</ReactMarkdown></div>}</>
})
