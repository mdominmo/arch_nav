# Concurrency Notes

This page captures practical rules for driver authors.

## Callback and lock ordering

- Do not invoke operation-complete callbacks while holding internal mutexes.
- Move/copy callback under lock, then invoke outside lock.

## Thread ownership

- Define a clear owner for each thread.
- `stop()` should handle repeated calls safely.
- Never `join()` from the same thread (`self-join`).

## Completion semantics

- Complete each operation exactly once.
- Protect completion paths against duplicate terminal signals.

## Shutdown

- Stop loop flags first.
- Unsubscribe external callbacks/subscriptions.
- Join or detach threads according to ownership and current thread context.
