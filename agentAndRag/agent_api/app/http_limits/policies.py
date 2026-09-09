"""Explicit HTTP route policies; resource concurrency is managed separately."""
from dataclasses import dataclass
import os

DEFAULTS = {
    "ip": (600, 120), "auth": (60, 20), "read": (300, 60),
    "write": (120, 20), "feedback": (30, 6), "generation": (60, 20),
    "control": (120, 20), "legacy": (30, 30),
}
EXEMPT = frozenset({"/health", "/ready", "/docs", "/openapi.json", "/redoc"})


def positive_env(name: str, default: int) -> int:
    value = int(os.getenv(name, str(default)))
    if value < 1:
        raise ValueError(f"{name} must be positive")
    return value


def configured_limits() -> dict[str, tuple[int, int]]:
    result = {}
    for name, (rate, burst) in DEFAULTS.items():
        if name == "generation":
            rate = positive_env("AGENT_PLATFORM_RATE_LIMIT", rate)
            burst = positive_env("AGENT_PLATFORM_RATE_BURST", burst)
        elif name == "legacy":
            rate = positive_env("AGENT_RATE_LIMIT", rate)
            burst = positive_env("AGENT_RATE_BURST", rate)
        result[name] = (positive_env(f"AGENT_HTTP_{name.upper()}_RATE", rate),
                        positive_env(f"AGENT_HTTP_{name.upper()}_BURST", burst))
    return result


@dataclass(frozen=True)
class Policy:
    group: str
    identity: str = "user"
    extra: str | None = None
    sensitive: tuple[int, int, int] | None = None  # user limit, seconds, IP limit


ROUTES: dict[tuple[str, str], Policy] = {}


def register(method: str, paths: str, policy: Policy) -> None:
    for path in paths.split():
        key = method, path
        if key in ROUTES:
            raise ValueError(f"duplicate rate policy: {key}")
        ROUTES[key] = policy


register("POST", " ".join("/api/v1/auth/" + name for name in (
    "login", "refresh", "logout", "invitations/accept", "password/forgot", "password/reset",
)), Policy("auth", "public"))
register("GET", "/api/v1/plans /api/v1/legal/{document_type}", Policy("read", "public"))
register("POST", "/api/v1/payments/test-webhook", Policy("write", "public"))
register("POST", "/api/v1/conversations/{conversation_id}/runs", Policy("generation"))
register("GET", "/api/v1/runs/{run_id} /api/v1/runs/{run_id}/events /api/v1/runs/{run_id}/experts", Policy("control"))
register("DELETE", "/api/v1/runs/{run_id}", Policy("control"))
register("PUT", "/api/v1/messages/{message_id}/feedback", Policy("write", extra="feedback"))
register("POST", "/api/v1/activation-codes/redeem", Policy("write", sensitive=(5, 600, 20)))
register("DELETE", "/api/v1/me/memories/{item_id:path}", Policy("write", sensitive=(5, 60, 30)))
register("DELETE", "/api/v1/me/memories", Policy("write", sensitive=(6, 3600, 20)))
register("GET", """
/api/v1/me /api/v1/me/api-keys /api/v1/me/preferences /api/v1/me/common-phrases /api/v1/me/memories
/api/v1/conversations /api/v1/conversations/search /api/v1/conversations/{conversation_id}
/api/v1/conversations/{conversation_id}/messages /api/v1/orders /api/v1/subscription /api/v1/credits
/api/v1/admin/overview /api/v1/admin/invitations /api/v1/admin/users /api/v1/admin/plans
/api/v1/admin/orders /api/v1/admin/subscriptions /api/v1/admin/api-keys /api/v1/admin/rate-limits
/api/v1/admin/runs /api/v1/admin/audit /api/v1/admin/users/{user_id}/data-snapshot
""", Policy("read"))
register("POST", """
/api/v1/conversations /api/v1/conversations/{conversation_id}/forks /api/v1/orders
/api/v1/me/api-keys /api/v1/me/common-phrases /api/v1/me/legal-acceptances /api/v1/feedback
/api/v1/admin/invitations /api/v1/admin/orders/{order_id}/confirm /api/v1/admin/users/{user_id}/credits
/api/v1/admin/users/{user_id}/data-snapshot/restore /api/v1/admin/users/{user_id}/data-snapshot/restore-file
""", Policy("write"))
register("PATCH", """
/api/v1/me /api/v1/me/preferences /api/v1/me/common-phrases/{phrase_id}
/api/v1/conversations/{conversation_id} /api/v1/admin/users/{user_id} /api/v1/admin/plans/{plan_code}
/api/v1/admin/orders/{order_id} /api/v1/admin/subscriptions/{subscription_id}
""", Policy("write"))
register("DELETE", """
/api/v1/conversations/{conversation_id} /api/v1/me/api-keys/{key_id} /api/v1/me/common-phrases/{phrase_id}
/api/v1/admin/invitations/{invitation_id} /api/v1/admin/api-keys/{key_id}
""", Policy("write"))
register("POST", "/v1/chat/completions", Policy("generation", "legacy"))
register("GET", "/v1/models", Policy("read", "legacy"))
register("GET", "/chat-moe /qa/history /qa/stats /qa/knowledge-gaps /qa/feedback-stats", Policy("legacy", "legacy"))
register("POST", "/chat-moe/completions", Policy("generation", "legacy"))
register("POST", "/chat-moe/sessions /qa/feedback", Policy("legacy", "legacy"))


def policy_for(method: str, template: str) -> Policy | None:
    if method == "OPTIONS" or template in EXEMPT:
        return None
    try:
        return ROUTES[method, template]
    except KeyError as exc:
        raise ValueError(f"HTTP route has no rate policy: {method} {template}") from exc
