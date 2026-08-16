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
  await expect(page.getByRole('button', { name: '退出登录' })).toBeVisible()

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
      { id: 'm-new-assistant', role: 'assistant', content: '已收到病例。', status: 'complete', created_at: '2026-08-14T08:01:00Z' },
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
  await expect(page.getByText('这是新会话的第一条病例信息')).toBeVisible()
  await expect(page.getByText('等待会诊资源')).toBeVisible()
  await expect(page.getByText('已收到病例。')).toBeVisible()
  await page.getByText('兽医临床专家').click()
  await expect(page.getByText('先排查尿道梗阻。')).toBeVisible()
  await expect(page.getByText(/仅展示任务结果，不展示系统提示词/)).toBeVisible()
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
