"""
Redis-backed distributed lock helpers for Celery tasks.

Use these helpers to prevent multiple workers from processing the same
business item (for example, the same message_id) at the same time.
"""

import logging
import secrets
from contextlib import contextmanager
from functools import wraps
from typing import Iterator, Optional

from django_redis import get_redis_connection

logger = logging.getLogger(__name__)

_UNLOCK_SCRIPT = """
if redis.call("GET", KEYS[1]) == ARGV[1] then
    return redis.call("DEL", KEYS[1])
else
    return 0
end
"""


def _release_lock(redis_client, lock_key: str, token: str) -> None:
    """Release lock only when the current owner token matches."""
    try:
        redis_client.eval(_UNLOCK_SCRIPT, 1, lock_key, token)
    except Exception:
        logger.exception("Failed to release Redis lock for key '%s'", lock_key)


@contextmanager
def redis_task_lock(lock_key: str, ttl_seconds: int = 300) -> Iterator[bool]:
    """
    Acquire a non-blocking Redis lock for a task execution.

    Args:
        lock_key: Unique lock key (include your business id in this key).
        ttl_seconds: Lock expiration in seconds for dead-worker safety.

    Yields:
        True when lock is acquired, False otherwise.
    """
    redis_client = get_redis_connection("default")
    token = secrets.token_hex(16)
    acquired = bool(redis_client.set(lock_key, token, nx=True, ex=ttl_seconds))

    try:
        yield acquired
    finally:
        if acquired:
            _release_lock(redis_client, lock_key, token)


def with_task_lock(
    key_template: str,
    key_arg: str = "message_id",
    ttl_seconds: int = 300,
    on_locked: str = "skip",
):
    """
    Decorate a Celery task function with a Redis lock.

    Args:
        key_template: Lock key template, e.g. "task_lock:process_message:{value}".
        key_arg: Name of the argument containing the business identifier.
        ttl_seconds: Lock expiration in seconds.
        on_locked: Behavior when lock exists: "skip" | "raise".

    Notes:
        - This decorator expects the decorated function to receive the business id
          as a keyword argument, or as the first positional argument.
        - Keep the task body idempotent even with this lock enabled.
    """

    if on_locked not in {"skip", "raise"}:
        raise ValueError("on_locked must be one of: 'skip', 'raise'")

    def decorator(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            value: Optional[object] = kwargs.get(key_arg)
            if value is None and args:
                value = args[0]
            if value is None:
                raise ValueError(
                    f"Could not build lock key. Missing value for argument '{key_arg}'."
                )

            lock_key = key_template.format(value=value)
            with redis_task_lock(lock_key=lock_key, ttl_seconds=ttl_seconds) as acquired:
                if not acquired:
                    logger.info(
                        "Task lock already held for key '%s'. Action=%s",
                        lock_key,
                        on_locked,
                    )
                    if on_locked == "raise":
                        raise RuntimeError(f"Task lock already held for key '{lock_key}'")
                    return None

                return func(*args, **kwargs)

        return wrapper

    return decorator
