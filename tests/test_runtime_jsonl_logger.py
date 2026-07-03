import json

from drone_autonomy.runtime.jsonl_logger import RuntimeJsonlLogger


def test_jsonl_logger_writes_one_record(tmp_path) -> None:
    log_path = tmp_path / "logs" / "runtime.jsonl"
    logger = RuntimeJsonlLogger(str(log_path))

    logger.write("mission_tick", phase="seek_gate", body_vx_m_s=0.1)
    logger.close()

    record = json.loads(log_path.read_text(encoding="utf-8").strip())
    assert record["event"] == "mission_tick"
    assert record["phase"] == "seek_gate"
    assert record["body_vx_m_s"] == 0.1
    assert "wall_time_s" in record


def test_jsonl_logger_empty_path_is_noop(tmp_path) -> None:
    logger = RuntimeJsonlLogger("")

    logger.write("ignored")
    logger.close()

    assert list(tmp_path.iterdir()) == []

