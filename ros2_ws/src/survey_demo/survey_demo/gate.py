"""The ready gate's conditions (pure; no ROS), unit-tested on the runner (see ``ready_gate``)."""

from __future__ import annotations


def parse_topic_minimum(item: str) -> tuple[str, int]:
    """``"/survey/capture:2"`` -> (``/survey/capture``, 2)."""
    topic, _, count = item.rpartition(":")
    if not topic or not count.isdigit():
        raise ValueError(f"topics_min_subscribers: expected '<topic>:<n>', got {item!r}")
    return topic, int(count)


def missing_items(names, services, subscriber_counts, want_nodes, want_services, want_topics,
                  want_flags, flags_received) -> list[str]:
    """What the gate still waits for (pure: the graph and the received topics as input)."""
    out = [f"node {n}" for n in want_nodes if n not in names]
    out += [f"service {s}" for s in want_services if s not in services]
    for topic, minimum in want_topics:
        count = subscriber_counts(topic)
        if count < minimum:
            out.append(f"topic {topic} ({count}/{minimum} subscriptions)")
    out += [f"message on {t}" for t in want_flags if t not in flags_received]
    return out
