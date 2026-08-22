import { expect, test, type Page } from '@playwright/test'

async function mockAuthenticated(page: Page, user: Record<string, string>) {
  await page.route('**/api/v1/auth/refresh', route => route.fulfill({ status: 200, contentType: 'application/json', body: JSON.stringify({ access_token: 'test-access-token', user }) }))
  await page.route('**/api/v1/me', route => route.fulfill({ status: 200, contentType: 'application/json', body: JSON.stringify(user) }))
}

test('login and public plan pages retain brand and keyboard focus', async ({ page }) => {
  await page.route('**/api/v1/me', route => route.fulfill({ status: 401, contentType: 'application/json', body: '{"code":"unauthorized"}' }))
  await page.route('**/api/v1/auth/refresh', route => route.fulfill({ status: 401, contentType: 'application/json', body: '{"code":"unauthorized"}' }))
  await page.route('**/api/v1/plans', route => route.fulfill({ status: 200, contentType: 'application/json', body: '{"items":[]}' }))
  await page.goto('/login')
  await expect(page.getByRole('heading', { name: /让判断更有依据/ })).toBeVisible()
  await expect(page.getByAltText('PetMind 团队标志').first()).toBeVisible()
  await page.keyboard.press('Tab')
  await expect(page.locator(':focus')).toBeVisible()
  await page.goto('/plans')
  await expect(page.getByRole('heading', { name: '按临床工作节奏选择' })).toBeVisible()
})

test('authenticated navigation exposes logout, audience mode and session details', async ({ page }) => {
  const user = { id: 'user-1', email: 'vet@petmind.local', display_name: '林医生', role: 'VET', status: 'active' }
  await mockAuthenticated(page, user)
  await page.route('**/api/v1/plans', route => route.fulfill({ status: 200, contentType: 'application/json', body: '{"items":[]}' }))
  await page.route('**/api/v1/conversations', route => route.fulfill({ status: 200, contentType: 'application/json', body: '{"items":[]}' }))
  await page.route('**/api/v1/auth/logout', route => route.fulfill({ status: 204, body: '' }))

  await page.goto('/plans')
  await expect(page.getByRole('button', { name: '关闭会员计划' })).toBeVisible()

  await page.goto('/chat')
  await page.getByRole('button', { name: '选择模型和回答身份' }).click()
  await expect(page.getByText('回答身份')).toBeVisible()
  const ownerMode = page.getByRole('button', { name: /宠物主/ })
  await ownerMode.click()
  await expect(ownerMode).toHaveClass(/selected/)
  await page.getByRole('button', { name: '选择模型和回答身份' }).click()
  await page.getByRole('button', { name: '会诊详情' }).click()
  await expect(page.getByRole('heading', { name: '当前会诊' })).toBeVisible()
  await expect(page.getByRole('definition').filter({ hasText: '宠物主沟通模式' })).toBeVisible()
  await expect(page.getByText('PetMind Clinical MoE').last()).toBeVisible()
})

test('membership uses the same standalone page for authenticated and public visitors', async ({ page }) => {
  test.skip((page.viewportSize()?.width || 0) <= 800, 'desktop persistent-sidebar behavior')
  const user = { id: 'user-1', email: 'vet@petmind.local', display_name: '林医生', role: 'VET', status: 'active' }
  let logoRequests = 0
  page.on('request', request => { if (request.url().includes('/brand/petmind-logo-cropped.png')) logoRequests += 1 })
  await mockAuthenticated(page, user)
  await page.route('**/api/v1/conversations', route => route.fulfill({ status: 200, contentType: 'application/json', body: '{"items":[]}' }))
  await page.route('**/api/v1/plans', route => route.fulfill({ status: 200, contentType: 'application/json', body: '{"items":[]}' }))

  await page.goto('/chat')
  await page.getByRole('link', { name: '积分' }).click()
  await expect(page).toHaveURL(/\/plans$/)
  await expect(page.locator('.workspace > .sidebar')).toHaveCount(0)
  await expect(page.getByRole('button', { name: '关闭会员计划' })).toBeVisible()
  expect(logoRequests).toBeGreaterThanOrEqual(1)
})

test('desktop sidebar collapses to an icon rail and expands again', async ({ page }) => {
  test.skip((page.viewportSize()?.width || 0) <= 800, 'desktop sidebar behavior')
  const user = { id: 'user-1', email: 'vet@petmind.local', display_name: '林医生', role: 'VET', status: 'active' }
  await mockAuthenticated(page, user)
  await page.route('**/api/v1/conversations', route => route.fulfill({ status: 200, contentType: 'application/json', body: '{"items":[]}' }))

  await page.goto('/chat')
  const workspace = page.locator('.workspace')
  await page.getByRole('button', { name: '收起侧边栏' }).click()
  await expect(workspace).toHaveClass(/sidebar-collapsed/)
  await expect(page.getByRole('button', { name: '展开侧边栏' })).toBeVisible()
  await expect(page.getByRole('button', { name: '新建会诊' })).toBeVisible()

  await page.getByRole('button', { name: '展开侧边栏' }).click()
  await expect(workspace).not.toHaveClass(/sidebar-collapsed/)
  await expect(page.getByRole('button', { name: '收起侧边栏' })).toBeVisible()
})

test('first message remains visible while a new conversation starts running', async ({ page }) => {
  const user = { id: 'user-1', email: 'vet@petmind.local', display_name: '林医生', role: 'VET', status: 'active' }
  const conversation = { id: 'case-new', title: '新会诊', status: 'active', created_at: '2026-08-14T08:00:00Z', last_active_at: '2026-08-14T08:00:00Z' }
  let runCompleted = false
  await mockAuthenticated(page, user)
  await page.route('**/api/v1/conversations', async route => {
    if (route.request().method() === 'POST') return route.fulfill({ status: 201, contentType: 'application/json', body: JSON.stringify(conversation) })
    return route.fulfill({ status: 200, contentType: 'application/json', body: '{"items":[]}' })
  })
  await page.route('**/api/v1/conversations/case-new/messages', async route => {
    await new Promise(resolve => setTimeout(resolve, 180))
    return route.fulfill({ status: 200, contentType: 'application/json', body: JSON.stringify({ items: runCompleted ? [
      { id: 'm-new-user', role: 'user', content: '这是新会话的第一条病例信息', status: 'complete', created_at: '2026-08-14T08:00:00Z' },
      { id: 'm-new-assistant', run_id: 'run-new', role: 'assistant', content: '已收到病例。', status: 'complete', created_at: '2026-08-14T08:01:00Z', expert_consultations: [{ expert: 'clinical', name: '兽医临床专家', status: 'completed', task: '评估急症风险', tools: [{ kind: 'tool', tool_name: 'rag.search', ok: true, latency_ms: 1200, result: { hits: 2, sources: ['book-a'] } }], opinion: { conclusion: '先排查尿道梗阻。', evidence: ['频繁蹲盆'], risks: ['尿闭风险'], confidence: 0.82 }, execution: 'single_pass' }] },
    ] : [] }) })
  })
  await page.route('**/api/v1/conversations/case-new/runs', async route => {
    await new Promise(resolve => setTimeout(resolve, 700))
    runCompleted = true
    return route.fulfill({
      status: 200,
      contentType: 'text/event-stream',
      headers: { 'X-PetMind-Run-Id': 'run-new' },
      body: 'id: 1\nevent: status\ndata: {"phase":"understanding"}\n\nid: 2\nevent: status\ndata: {"phase":"consulting","agent_status":"expert_calling","expert":{"expert":"clinical","name":"兽医临床专家","status":"running"}}\n\nid: 3\nevent: status\ndata: {"phase":"consulting","agent_status":"expert_complete","expert":{"expert":"clinical","name":"兽医临床专家","status":"completed","task":"评估急症风险","tools":[{"kind":"tool","tool_name":"rag.search","ok":true,"latency_ms":1200,"result":{"hits":2,"sources":["book-a"]}}],"opinion":{"conclusion":"先排查尿道梗阻。","evidence":["频繁蹲盆"],"risks":["尿闭风险"],"confidence":0.82},"execution":"single_pass"}}\n\nid: 4\nevent: delta\ndata: {"content":"已收到病例。"}\n\nid: 5\nevent: completed\ndata: {"credits":1,"finish_reason":"stop"}\n\n',
    })
  })

  await page.goto('/chat')
  await page.getByRole('textbox', { name: '输入病例' }).fill('这是新会话的第一条病例信息')
  await page.getByRole('button', { name: '发送' }).click()
  await expect(page).toHaveURL(/\/chat\/case-new$/)
  await expect(page.locator('.message.user').getByText('这是新会话的第一条病例信息')).toBeVisible()
  expect(runCompleted).toBe(false)
  const mobile = (page.viewportSize()?.width || 1000) <= 800
  if (mobile) await page.getByRole('button', { name: '打开侧边栏' }).click()
  await expect(page.locator('.sidebar:visible .history-main').filter({ hasText: '这是新会话的第一条病例信息' })).toBeVisible()
  if (mobile) await page.locator('.sidebar:visible').getByRole('button', { name: '收起侧边栏' }).click()
  await expect(page.getByText('等待会诊资源', { exact: true })).toBeVisible()
  await expect(page.getByText('已收到病例。')).toBeVisible()
  await page.locator('.message.assistant .expert-consultation-heading').click()
  await expect(page.getByText('先排查尿道梗阻。')).toBeVisible()
  await expect(page.getByText(/仅展示任务结果，不展示系统提示词/)).toBeVisible()
})

test('switching conversations shows a loading marker instead of stale messages', async ({ page }) => {
  const user = { id: 'user-1', email: 'vet@petmind.local', display_name: '林医生', role: 'VET', status: 'active' }
  const conversations = [
    { id: 'case-a', title: '病例 A', status: 'active', created_at: '2026-08-14T08:00:00Z', last_active_at: '2026-08-14T08:00:00Z' },
    { id: 'case-b', title: '病例 B', status: 'active', created_at: '2026-08-14T09:00:00Z', last_active_at: '2026-08-14T09:00:00Z' },
  ]
  await mockAuthenticated(page, user)
  await page.route('**/api/v1/conversations', route => route.fulfill({ status: 200, contentType: 'application/json', body: JSON.stringify({ items: conversations }) }))
  await page.route('**/api/v1/conversations/case-a/messages', route => route.fulfill({ status: 200, contentType: 'application/json', body: '{"items":[{"id":"a1","role":"user","content":"病例 A 的旧内容","status":"complete","created_at":"2026-08-14T08:00:00Z"}]}' }))
  await page.route('**/api/v1/conversations/case-b/messages', async route => {
    await new Promise(resolve => setTimeout(resolve, 500))
    return route.fulfill({ status: 200, contentType: 'application/json', body: '{"items":[{"id":"b1","role":"user","content":"病例 B 的内容","status":"complete","created_at":"2026-08-14T09:00:00Z"}]}' })
  })

  await page.goto('/chat/case-a')
  await expect(page.getByText('病例 A 的旧内容')).toBeVisible()
  if ((page.viewportSize()?.width || 1000) <= 800) await page.getByRole('button', { name: '打开侧边栏' }).click()
  await page.locator('.sidebar:visible .history-main').filter({ hasText: '病例 B' }).click()
  await expect(page.getByRole('status', { name: '正在加载对话' })).toBeVisible()
  await expect(page.getByText('病例 A 的旧内容')).toHaveCount(0)
  await expect(page.getByText('病例 B 的内容')).toBeVisible()
})

test('collapsed agent and expert cards stay before the streaming answer', async ({ page }) => {
  const user = { id: 'user-1', email: 'vet@petmind.local', display_name: '林医生', role: 'VET', status: 'active' }
  const conversation = { id: 'case-layout', title: '布局测试', status: 'active', created_at: '2026-08-14T08:00:00Z', last_active_at: '2026-08-14T08:00:00Z' }
  await mockAuthenticated(page, user)
  await page.route('**/api/v1/conversations', async route => route.request().method() === 'POST'
    ? route.fulfill({ status: 201, contentType: 'application/json', body: JSON.stringify(conversation) })
    : route.fulfill({ status: 200, contentType: 'application/json', body: '{"items":[]}' }))
  await page.route('**/api/v1/conversations/case-layout/runs', route => route.fulfill({
    status: 200,
    contentType: 'text/event-stream',
    headers: { 'X-PetMind-Run-Id': 'run-layout' },
    body: 'id: 1\nevent: status\ndata: {"phase":"consulting","expert":{"expert":"clinical","name":"兽医临床专家","status":"running"}}\n\nid: 2\nevent: delta\ndata: {"content":"正在形成回答"}\n\n',
  }))

  await page.goto('/chat')
  await page.getByRole('textbox', { name: '输入病例' }).fill('检查生成布局')
  await page.getByRole('button', { name: '发送' }).click()
  const answer = page.locator('.message.assistant')
  await expect(answer.locator('.expert-consultation')).toBeVisible()
  await expect(answer.getByText('Agent 处理流程')).toBeVisible()
  await expect(answer.locator('.expert-consultation-heading strong')).toHaveText('专家会诊')
  await expect(answer.getByText('正在形成回答')).toBeVisible()
  expect(await answer.evaluate(node => {
    const process = node.querySelector('.agent-process-flow')!
    const expert = node.querySelector('.expert-panel')!
    const body = node.querySelector('.message-body')!
    return Boolean((process.compareDocumentPosition(expert) & Node.DOCUMENT_POSITION_FOLLOWING)
      && (expert.compareDocumentPosition(body) & Node.DOCUMENT_POSITION_FOLLOWING))
  })).toBe(true)
})

test('an interrupted final answer is replaced after a durable reset event', async ({ page }) => {
  const user = { id: 'user-1', email: 'vet@petmind.local', display_name: '林医生', role: 'VET', status: 'active' }
  const conversation = { id: 'case-reset', title: '终答重试测试', status: 'active', created_at: '2026-08-14T08:00:00Z', last_active_at: '2026-08-14T08:00:00Z' }
  await mockAuthenticated(page, user)
  await page.route('**/api/v1/conversations', async route => route.request().method() === 'POST'
    ? route.fulfill({ status: 201, contentType: 'application/json', body: JSON.stringify(conversation) })
    : route.fulfill({ status: 200, contentType: 'application/json', body: '{"items":[]}' }))
  await page.route('**/api/v1/conversations/case-reset/messages', route => route.fulfill({
    status: 200,
    contentType: 'application/json',
    body: '{"items":[{"id":"m-final","run_id":"run-reset","role":"assistant","content":"完整替代回答","status":"complete","created_at":"2026-08-14T08:01:00Z","expert_consultations":[]}]}',
  }))
  await page.route('**/api/v1/conversations/case-reset/runs', route => route.fulfill({
    status: 200,
    contentType: 'text/event-stream',
    headers: { 'X-PetMind-Run-Id': 'run-reset' },
    body: 'id: 1\nevent: delta\ndata: {"content":"不完整片段"}\n\nid: 2\nevent: reset\ndata: {"reason":"aggregator_stream_interrupted"}\n\nid: 3\nevent: delta\ndata: {"content":"完整替代回答"}\n\nid: 4\nevent: completed\ndata: {"credits":2,"finish_reason":"stop"}\n\n',
  }))

  await page.goto('/chat')
  await page.getByRole('textbox', { name: '输入病例' }).fill('检查终答重试')
  await page.getByRole('button', { name: '发送' }).click()
  await expect(page.getByText('完整替代回答')).toBeVisible()
  await expect(page.getByText('不完整片段')).toHaveCount(0)
})

test('conversation history can be exported and deleted', async ({ page }) => {
  const user = { id: 'user-1', email: 'vet@petmind.local', display_name: '林医生', role: 'VET', status: 'active' }
  const conversation = { id: 'case-1', title: '猫咪复诊记录', status: 'active', created_at: '2026-08-14T08:00:00Z', last_active_at: '2026-08-14T08:30:00Z' }
  let deleted = false
  await mockAuthenticated(page, user)
  await page.route('**/api/v1/conversations', route => route.fulfill({ status: 200, contentType: 'application/json', body: JSON.stringify({ items: [conversation] }) }))
  await page.route('**/api/v1/conversations/case-1/messages', route => route.fulfill({ status: 200, contentType: 'application/json', body: JSON.stringify({ items: [
    { id: 'm1', role: 'user', content: '猫咪今天食欲下降。', status: 'complete', created_at: '2026-08-14T08:00:00Z' },
    { id: 'm2', role: 'assistant', content: '请先确认精神状态和饮水情况。', status: 'complete', created_at: '2026-08-14T08:01:00Z' },
  ] }) }))
  await page.route('**/api/v1/conversations/case-1', route => {
    if (route.request().method() === 'DELETE') { deleted = true; return route.fulfill({ status: 204, body: '' }) }
    return route.fallback()
  })

  await page.goto('/chat')
  if ((page.viewportSize()?.width || 1000) <= 800) await page.getByRole('button', { name: '打开侧边栏' }).click()
  const actions = page.locator('button[aria-label="管理对话：猫咪复诊记录"]:visible')
  await actions.click()
  const downloadPromise = page.waitForEvent('download')
  await page.getByRole('menuitem', { name: '导出 Markdown' }).click()
  const download = await downloadPromise
  expect(download.suggestedFilename()).toBe('猫咪复诊记录.md')

  await actions.click()
  page.once('dialog', dialog => dialog.accept())
  await page.getByRole('menuitem', { name: '删除对话' }).click()
  await expect(page.locator('button[aria-label="管理对话：猫咪复诊记录"]:visible')).toHaveCount(0)
  expect(deleted).toBe(true)
})

test('message actions copy content and persist assistant feedback and forks', async ({ page }) => {
  const user = { id: 'user-1', email: 'vet@petmind.local', display_name: '林医生', role: 'VET', status: 'active' }
  const conversation = { id: 'case-actions', title: '消息操作测试', status: 'active', created_at: '2026-08-23T08:00:00Z', last_active_at: '2026-08-23T08:30:00Z' }
  const forked = { id: 'case-actions-fork', title: '消息操作测试 · 分支', status: 'active', created_at: '2026-08-23T09:00:00Z', last_active_at: '2026-08-23T09:00:00Z', source_conversation_id: 'case-actions', forked_from_message_id: 'message-assistant', copied_messages: 2 }
  let feedback: 'up' | 'down' | null = null
  let forkedRequest = false
  await page.context().grantPermissions(['clipboard-read', 'clipboard-write'])
  await mockAuthenticated(page, user)
  await page.route('**/api/v1/conversations', route => route.fulfill({ status: 200, contentType: 'application/json', body: JSON.stringify({ items: [conversation] }) }))
  await page.route('**/api/v1/conversations/case-actions/messages', route => route.fulfill({ status: 200, contentType: 'application/json', body: JSON.stringify({ items: [
    { id: 'message-user', role: 'user', content: '猫咪频繁蹲猫砂盆。', status: 'complete', created_at: '2026-08-23T08:00:00Z' },
    { id: 'message-assistant', run_id: 'run-actions', role: 'assistant', content: '首先排除尿道梗阻。', status: 'complete', created_at: '2026-08-23T08:01:00Z', feedback_rating: feedback, expert_consultations: [], trace_nodes: [] },
  ] }) }))
  await page.route('**/api/v1/messages/message-assistant/feedback', async route => {
    const body = route.request().postDataJSON() as {rating: 'up' | 'down' | null}
    feedback = body.rating
    return route.fulfill({ status: 200, contentType: 'application/json', body: JSON.stringify({ message_id: 'message-assistant', rating: feedback, updated_at: feedback ? '2026-08-23T09:01:00Z' : null }) })
  })
  await page.route('**/api/v1/conversations/case-actions/forks', route => {
    forkedRequest = true
    expect(route.request().headers()['idempotency-key']).toBeTruthy()
    return route.fulfill({ status: 201, contentType: 'application/json', body: JSON.stringify(forked) })
  })
  await page.route('**/api/v1/conversations/case-actions-fork/messages', route => route.fulfill({ status: 200, contentType: 'application/json', body: JSON.stringify({ items: [
    { id: 'fork-user', role: 'user', content: '猫咪频繁蹲猫砂盆。', status: 'complete', created_at: '2026-08-23T08:00:00Z' },
    { id: 'fork-assistant', run_id: 'run-actions', role: 'assistant', content: '首先排除尿道梗阻。', status: 'complete', created_at: '2026-08-23T08:01:00Z', feedback_rating: null, expert_consultations: [], trace_nodes: [] },
  ] }) }))
  await page.goto('/chat/case-actions')
  const userMessage = page.locator('.message.user')
  await userMessage.getByRole('button', { name: '复制消息' }).click()
  await expect(userMessage.getByRole('button', { name: '已复制' })).toBeVisible()
  await expect(userMessage).toHaveCSS('margin-left', /.+/)
  const assistant = page.locator('.message.assistant')
  await expect(assistant.getByText('首先排除尿道梗阻。')).toBeVisible()
  await assistant.getByRole('button', { name: '复制消息' }).click()
  await expect(assistant.getByRole('button', { name: '已复制' })).toBeVisible()
  expect(await page.evaluate(() => navigator.clipboard.readText())).toContain('首先排除尿道梗阻。')

  const like = assistant.getByRole('button', { name: '赞同回答', exact: true })
  await like.click()
  await expect(like).toHaveAttribute('aria-pressed', 'true')
  expect(feedback).toBe('up')
  await like.click()
  await expect(like).toHaveAttribute('aria-pressed', 'false')
  expect(feedback).toBeNull()
  const dislike = assistant.getByRole('button', { name: '不赞同回答' })
  await dislike.click()
  await expect(dislike).toHaveAttribute('aria-pressed', 'true')
  expect(feedback).toBe('down')

  await assistant.getByRole('button', { name: '从此消息创建分支' }).click()
  await expect(page).toHaveURL(/\/chat\/case-actions-fork$/)
  await expect(page.getByText('首先排除尿道梗阻。')).toBeVisible()
  expect(forkedRequest).toBe(true)
  await expect(page.getByRole('button', { name: '删除当前对话' })).toHaveCount(0)
})

test('editing a user message truncates later UI and persists the replacement branch', async ({ page }) => {
  const user = { id: 'user-1', email: 'vet@petmind.local', display_name: '林医生', role: 'VET', status: 'active' }
  const conversation = { id: 'case-rewrite', title: '消息编辑测试', status: 'active', created_at: '2026-08-23T08:00:00Z', last_active_at: '2026-08-23T08:30:00Z' }
  let items = [
    { id: 'message-user-first', role: 'user', content: '猫咪频繁蹲猫砂盆。', status: 'complete', created_at: '2026-08-23T08:00:00Z' },
    { id: 'message-assistant-first', run_id: 'run-first', role: 'assistant', content: '旧回答：首先排除尿道梗阻。', status: 'complete', created_at: '2026-08-23T08:01:00Z', expert_consultations: [], trace_nodes: [] },
    { id: 'message-user-later', role: 'user', content: '它目前还能排一点尿。', status: 'complete', created_at: '2026-08-23T08:02:00Z' },
    { id: 'message-assistant-later', run_id: 'run-later', role: 'assistant', content: '旧回答：继续观察。', status: 'complete', created_at: '2026-08-23T08:03:00Z', expert_consultations: [], trace_nodes: [] },
  ]
  let submitted: Record<string, unknown> | null = null
  await mockAuthenticated(page, user)
  await page.route('**/api/v1/conversations', route => route.fulfill({ status: 200, contentType: 'application/json', body: JSON.stringify({ items: [conversation] }) }))
  await page.route('**/api/v1/conversations/case-rewrite/messages', route => route.fulfill({
    status: 200,
    contentType: 'application/json',
    body: JSON.stringify({ items }),
  }))
  await page.route('**/api/v1/conversations/case-rewrite/runs', route => {
    submitted = route.request().postDataJSON() as Record<string, unknown>
    items = [{
      id: 'message-user-replacement',
      role: 'user',
      content: String(submitted.message),
      status: 'complete',
      created_at: '2026-08-23T08:00:00Z',
    }, {
      id: 'message-assistant-replacement',
      run_id: 'run-rewrite',
      role: 'assistant',
      content: '新回答：按尿道梗阻急症分诊。',
      status: 'complete',
      created_at: '2026-08-23T08:01:00Z',
      expert_consultations: [],
      trace_nodes: [],
    }]
    return route.fulfill({
      status: 200,
      contentType: 'text/event-stream',
      headers: { 'X-PetMind-Run-Id': 'run-rewrite' },
      body: 'id: 1\nevent: status\ndata: {"phase":"generating"}\n\nid: 2\nevent: delta\ndata: {"content":"新回答：按尿道梗阻急症分诊。"}\n\nid: 3\nevent: completed\ndata: {}\n\n',
    })
  })

  await page.goto('/chat/case-rewrite')
  const firstUser = page.locator('.message.user').first()
  const userBubble = firstUser.locator('.user-message-bubble')
  const actions = firstUser.locator('.message-actions')
  await expect(actions).toHaveCSS('justify-content', 'flex-end')
  expect(await firstUser.evaluate(node => {
    const bubble = node.querySelector('.user-message-bubble')
    const lastAction = node.querySelector('.message-actions button:last-of-type')
    if (!bubble || !lastAction) return false
    return Math.abs(bubble.getBoundingClientRect().right - lastAction.getBoundingClientRect().right) < 8
  })).toBe(true)

  await firstUser.getByRole('button', { name: '编辑消息' }).click()
  const editor = firstUser.getByRole('textbox', { name: '编辑用户消息' })
  await expect(editor).toBeVisible()
  await expect(page.getByRole('textbox', { name: '输入病例' })).toHaveValue('')
  await editor.fill('猫频繁蹲盆且只能排出几滴尿，如何排急症？')
  await firstUser.getByRole('button', { name: '发送', exact: true }).click()

  await expect(page.getByText('旧回答：首先排除尿道梗阻。')).toHaveCount(0)
  await expect(page.getByText('它目前还能排一点尿。')).toHaveCount(0)
  await expect(page.getByText('旧回答：继续观察。')).toHaveCount(0)
  await expect(page.getByText('猫频繁蹲盆且只能排出几滴尿，如何排急症？')).toBeVisible()
  await expect(page.getByText('新回答：按尿道梗阻急症分诊。')).toBeVisible()
  expect(submitted).toMatchObject({
    message: '猫频繁蹲盆且只能排出几滴尿，如何排急症？',
    rewrite_message_id: 'message-user-first',
    delivery: 'sse',
  })
  await expect(userBubble).toHaveCount(1)
})

test('API key revocation confirms, persists and replaces the action with status', async ({ page }) => {
  const user = { id: 'user-1', email: 'vet@petmind.local', display_name: '林医生', role: 'VET', status: 'active' }
  let revokedAt: string | null = null
  await mockAuthenticated(page, user)
  await page.route('**/api/v1/credits', route => route.fulfill({ status: 200, contentType: 'application/json', body: '{"balance":100,"reserved":0,"ledger":[]}' }))
  await page.route('**/api/v1/subscription', route => route.fulfill({ status: 200, contentType: 'application/json', body: '{"subscription":null}' }))
  await page.route('**/api/v1/orders', route => route.fulfill({ status: 200, contentType: 'application/json', body: '{"items":[]}' }))
  await page.route('**/api/v1/me/api-keys', route => route.fulfill({ status: 200, contentType: 'application/json', body: JSON.stringify({ items: [{ id: 'key-1', name: '诊所系统', prefix: 'abcdef12', scopes: ['chat:write'], expires_at: null, revoked_at: revokedAt, last_used_at: null, created_at: '2026-08-14T08:00:00Z' }] }) }))
  await page.route('**/api/v1/me/api-keys/key-1', route => {
    revokedAt = '2026-08-14T09:00:00Z'
    return route.fulfill({ status: 204, body: '' })
  })

  await page.goto('/settings#api-keys')
  page.once('dialog', dialog => dialog.accept())
  await page.getByRole('button', { name: '撤销', exact: true }).click()
  await expect(page.getByText(/原密钥已立即失效/)).toBeVisible()
  await expect(page.getByText(/已撤销 ·/)).toBeVisible()
  await expect(page.getByRole('button', { name: '撤销', exact: true })).toHaveCount(0)
})

test('an in-flight consultation resumes after refresh and clears transient recovery errors', async ({ page }) => {
  const user = { id: 'user-1', email: 'vet@petmind.local', display_name: '林医生', role: 'VET', status: 'active' }
  const conversation = { id: 'case-resume', title: '恢复中的会诊', status: 'active', created_at: '2026-08-14T08:00:00Z', last_active_at: '2026-08-14T08:30:00Z' }
  await page.addInitScript(() => sessionStorage.setItem('petmind-run:case-resume', JSON.stringify({ runId: 'run-resume', lastEvent: 1 })))
  await mockAuthenticated(page, user)
  await page.route('**/api/v1/conversations', route => route.fulfill({ status: 200, contentType: 'application/json', body: JSON.stringify({ items: [conversation] }) }))
  await page.route('**/api/v1/conversations/case-resume/messages', route => route.fulfill({ status: 200, contentType: 'application/json', body: JSON.stringify({ items: [
    { id: 'm1', role: 'user', content: '刷新前的问题', status: 'complete', created_at: '2026-08-14T08:00:00Z' },
    { id: 'm2', role: 'assistant', content: '数据库中的最终回答', status: 'complete', created_at: '2026-08-14T08:01:00Z' },
  ] }) }))
  await page.route('**/api/v1/runs/run-resume/events', route => route.fulfill({
    status: 200,
    contentType: 'text/event-stream',
    body: 'id: 2\nevent: delta\ndata: {"content":"恢复的流式片段"}\n\nid: 3\nevent: completed\ndata: {"credits":2}\n\n',
  }))

  await page.goto('/chat/case-resume')
  await expect(page.getByText('数据库中的最终回答')).toBeVisible()
  await expect(page.getByText('会诊流恢复失败，请稍后重试')).toHaveCount(0)
  await expect.poll(() => page.evaluate(() => sessionStorage.getItem('petmind-run:case-resume'))).toBeNull()
})

test('historical expert opinions stay attached before their assistant answer', async ({ page }) => {
  const user = { id: 'user-1', email: 'vet@petmind.local', display_name: '林医生', role: 'VET', status: 'active' }
  const conversation = { id: 'case-experts', title: '猫泌尿会诊', status: 'active', created_at: '2026-08-16T08:00:00Z', last_active_at: '2026-08-16T08:30:00Z' }
  const expert = {
    expert: 'clinical', name: '兽医临床专家', status: 'completed', task: '核对泌尿急症', execution: 'single_pass',
    tools: [{ kind: 'tool', tool_name: 'rag.search', ok: true, latency_ms: 120, result: { hits: 3, sources: ['books/058.mmd'] } }],
    opinion: { conclusion: '优先排查尿道梗阻', evidence: ['频繁蹲盆且尿量少'], risks: ['完全尿闭属于急症'], confidence: 0.91 },
  }
  await mockAuthenticated(page, user)
  await page.route('**/api/v1/conversations', route => route.fulfill({ status: 200, contentType: 'application/json', body: JSON.stringify({ items: [conversation] }) }))
  await page.route('**/api/v1/conversations/case-experts/messages', route => route.fulfill({ status: 200, contentType: 'application/json', body: JSON.stringify({ items: [
    { id: 'm1', run_id: null, role: 'user', content: '猫频繁蹲盆', status: 'complete', created_at: '2026-08-16T08:00:00Z', expert_consultations: [] },
    { id: 'm2', run_id: 'run-experts', role: 'assistant', content: '请先判断是否存在完全尿闭。', status: 'complete', created_at: '2026-08-16T08:01:00Z', expert_consultations: [expert] },
  ] }) }))

  await page.goto('/chat/case-experts')
  const answer = page.locator('.message.assistant')
  await expect(answer.getByText('专家会诊')).toBeVisible()
  await answer.locator('.expert-consultation-heading').click()
  await expect(answer.getByText('优先排查尿道梗阻')).toBeVisible()
  await expect(answer.getByText('请先判断是否存在完全尿闭。')).toBeVisible()
  expect(await answer.evaluate(node => {
    const expertPanel = node.querySelector('.expert-consultation')
    const body = node.querySelector('.message-body')
    return Boolean(expertPanel && body && (expertPanel.compareDocumentPosition(body) & Node.DOCUMENT_POSITION_FOLLOWING))
  })).toBe(true)
  await expect(page.locator('.messages > .expert-consultation')).toHaveCount(0)
})

test('settings manages phrases, provenance-visible memory and activation navigation', async ({ page }) => {
  const user = { id: 'user-1', email: 'vet@petmind.local', display_name: '林医生', role: 'VET', status: 'active' }
  let phrases: Array<{id: string; title: string; content: string; sort_order: number}> = []
  const clearedScopes: string[] = []
  await mockAuthenticated(page, user)
  await page.route('**/api/v1/me/preferences', async route => {
    const current = { theme: 'system', locale: 'zh-CN', default_expand_experts: true, memory_recall_enabled: true, memory_write_enabled: true }
    return route.fulfill({ status: 200, contentType: 'application/json', body: JSON.stringify(route.request().method() === 'PATCH' ? { ...current, ...route.request().postDataJSON() } : current) })
  })
  await page.route('**/api/v1/me/common-phrases', async route => {
    if (route.request().method() === 'POST') {
      phrases = [{ id: 'phrase-1', ...route.request().postDataJSON() }]
      return route.fulfill({ status: 201, contentType: 'application/json', body: JSON.stringify(phrases[0]) })
    }
    return route.fulfill({ status: 200, contentType: 'application/json', body: JSON.stringify({ items: phrases }) })
  })
  await page.route('**/api/v1/me/api-keys', route => route.fulfill({ status: 200, contentType: 'application/json', body: '{"items":[]}' }))
  await page.route('**/api/v1/credits', route => route.fulfill({ status: 200, contentType: 'application/json', body: '{"balance":999,"reserved":0}' }))
  await page.route('**/api/v1/me/memories', route => {
    if (route.request().method() === 'DELETE') {
      clearedScopes.push(route.request().postDataJSON().scope)
      return route.fulfill({ status: 200, contentType: 'application/json', body: '{"deleted":{"items":1}}' })
    }
    return route.fulfill({ status: 200, contentType: 'application/json', body: JSON.stringify({ items: [
    { id: 'short_term:s1', type: 'short_term', label: '近期对话', content: '团团今天抓挠增加', created_at: '2026-08-20T08:00:00Z', source_count: 0, generation_tags: [] },
    { id: 'profile:petFacts.团团.年龄', type: 'profile', label: 'petFacts.团团.年龄', content: '4岁', source_count: 1, generation_tags: [] },
    { id: 'profile:petFacts.团团.品种', type: 'profile', label: 'petFacts.团团.品种', content: '英短', source_count: 1, generation_tags: [] },
    { id: 'knowledge:k1', type: 'knowledge', label: 'extraction', content: '团团对鸡肉蛋白过敏', source_count: 3, generation_tags: ['knowledge_extraction'] },
  ] }) })
  })

  await page.goto('/settings')
  await expect(page.getByRole('heading', { name: '设置' })).toBeVisible()
  await page.locator('.settings-account').click()
  await expect(page.getByRole('heading', { name: '账号设置' })).toBeVisible()
  await expect(page.getByLabel('当前头像')).toHaveText('林')
  await expect(page.getByLabel('显示名称')).toHaveValue('林医生')
  await expect(page.getByLabel('登录邮箱')).toHaveValue('vet@petmind.local')
  await expect(page.getByRole('button', { name: /删除账号/ })).toHaveCount(0)
  await page.getByRole('button', { name: '关闭设置弹窗' }).click()
  await page.getByRole('button', { name: /常用语/ }).click()
  await page.getByPlaceholder('名称（可选）').fill('过敏复诊')
  await page.getByPlaceholder('输入常用病例描述或要求').fill('请对比皮损和瘙痒评分变化')
  await page.getByRole('button', { name: '保存常用语' }).click()
  await expect(page.getByText('过敏复诊')).toBeVisible()
  await page.getByRole('button', { name: '关闭设置弹窗' }).click()

  await page.getByRole('button', { name: '记忆 查看与管理' }).click()
  await expect(page.getByRole('heading', { name: '短期记忆 1' })).toBeVisible()
  await expect(page.getByRole('heading', { name: '长期记忆 1' })).toBeVisible()
  await expect(page.getByRole('heading', { name: '用户画像 2' })).toBeVisible()
  await expect(page.getByLabel('记忆条目列表')).toHaveCSS('overflow-y', 'auto')
  await expect(page.getByText('团团今天抓挠增加')).toBeVisible()
  await expect(page.getByText('团团对鸡肉蛋白过敏')).toBeVisible()
  await expect(page.getByText('用户画像')).toHaveCount(1)
  await expect(page.getByText('团团 · 年龄')).toBeVisible()
  await expect(page.getByText(/中期主题来源/)).toHaveCount(0)
  await expect(page.getByPlaceholder('输入当前密码')).toHaveCount(0)
  for (const name of ['清空短期记忆', '清空长期记忆', '清空用户画像']) {
    page.once('dialog', dialog => dialog.accept())
    await page.getByRole('button', { name }).click()
  }
  expect(clearedScopes).toEqual(['short_term', 'knowledge', 'profile'])
  await expect(page.getByRole('button', { name: /隐藏/ })).toHaveCount(0)
  await page.getByRole('button', { name: '关闭设置弹窗' }).click()

  await page.getByRole('button', { name: /邀请码激活/ }).click()
  await expect(page).toHaveURL(/\/activate-code$/)
  await expect(page.getByRole('heading', { name: '邀请码激活' })).toBeVisible()
})
