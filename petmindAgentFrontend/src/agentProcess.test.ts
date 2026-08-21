import { describe, expect, it } from 'vitest'
import { agentProcessSummary } from './agentProcess'
import type { TraceNode } from './types'

function node(overrides: Partial<TraceNode>): TraceNode {
  return {
    version: 1,
    node_id: 'node-1',
    parent_id: '',
    node_type: 'query',
    status: 'running',
    wave: 2,
    goal_id: 'goal-1',
    details: { tool_name: 'mcp.web_search.web_search', query: 'canine guideline' },
    ...overrides,
  }
}

describe('agentProcessSummary', () => {
  it('折叠时优先展示当前真实执行节点', () => {
    expect(agentProcessSummary([node({})], 'consulting', true)).toBe('第 2 波 · 网络补证 · 执行中')
  })

  it('尚无轨迹节点时展示当前公开阶段', () => {
    expect(agentProcessSummary([], 'routing', true)).toBe('组织会诊路径')
  })

  it('历史消息展示完成步骤数或降级状态', () => {
    expect(agentProcessSummary([
      node({ node_id: 'decision', node_type: 'decision', status: 'completed' }),
      node({ node_id: 'answer', node_type: 'answer', status: 'completed' }),
    ], '', false)).toBe('处理完成 · 2 个步骤')
    expect(agentProcessSummary([
      node({ status: 'degraded' }),
    ], '', false)).toBe('处理完成 · 部分证据不足')
  })
})
