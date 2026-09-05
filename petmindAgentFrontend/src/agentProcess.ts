import type { TraceNode } from './types'

export const phaseLabels: Record<string, string> = {
  queued: '等待会诊资源',
  cancelling: '正在停止生成',
  reconnecting: '正在恢复会诊流',
  understanding: '理解问题',
  routing: '组织会诊路径',
  consulting: '专家会诊',
  reviewing: '安全复核',
  generating: '整理答复',
}

export function traceNodeTitle(node: TraceNode): string {
  const detail = node.details || {}
  if (node.node_type === 'decision') return `${detail.intent_id || '任务'} · ${detail.intent_name || '意图识别'}`
  if (node.node_type === 'goal') return '证据目标'
  if (node.node_type === 'query') {
    const tool = detail.tool_name === 'rag.search'
      ? '本地知识检索'
      : detail.tool_name?.includes('web_search') ? '网络补证' : detail.tool_name || '资料核对'
    return `${node.wave > 1 ? `第 ${node.wave} 波 · ` : ''}${tool}`
  }
  if (node.node_type === 'review') return '安全复核'
  if (node.node_type === 'answer') return '整理答复'
  return '专家会诊'
}

export function agentProcessSummary(nodes: TraceNode[], phase: string, active: boolean): string {
  const visible = nodes.filter(node => node.node_type !== 'expert')
  const running = [...visible].reverse().find(node => node.status === 'running')
  if (running) return `${traceNodeTitle(running)} · 执行中`
  if (active) return phaseLabels[phase] || '正在处理'
  if (visible.some(node => node.status === 'failed')) return '处理结束 · 存在失败步骤'
  if (visible.some(node => node.status === 'degraded')) return '处理完成 · 部分证据不足'
  if (visible.length) return `处理完成 · ${visible.length} 个步骤`
  return '等待处理记录'
}
