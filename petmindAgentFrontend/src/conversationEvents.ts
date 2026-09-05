import type { Conversation } from './types'

export const CONVERSATION_UPSERT_EVENT = 'petmind:conversation-upsert'
export const CONVERSATION_DELETE_EVENT = 'petmind:conversation-delete'
export type ConversationUpdate = Partial<Conversation> & Pick<Conversation, 'id'>

export function announceConversation(update: ConversationUpdate) {
  window.dispatchEvent(new CustomEvent<ConversationUpdate>(CONVERSATION_UPSERT_EVENT, { detail: update }))
}

export function announceConversationDeleted(id: string) {
  window.dispatchEvent(new CustomEvent<string>(CONVERSATION_DELETE_EVENT, { detail: id }))
}
