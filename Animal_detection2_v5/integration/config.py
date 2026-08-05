import os


def _get_env(name: str, default: str = "") -> str:
    value = os.getenv(name, default)
    return value.strip()


N8N_WEBHOOK_URL = _get_env("N8N_WEBHOOK_URL", "http://localhost:5678/webhook/a3ab4cd2-f1f7-4a49-8c66-0669554731ec")
N8N_TIMEOUT_SEC = float(_get_env("N8N_TIMEOUT_SEC", "10"))
