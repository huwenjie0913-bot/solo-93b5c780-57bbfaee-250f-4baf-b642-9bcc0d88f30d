"""刀具寿命预测：数据校验、磨损趋势、剩余寿命估算与 API。"""

import json

import pytest

from gcode_review.app import create_app
from gcode_review.tool_life import (LifeValidationError, predict_tool_life,
                                    validate_records)


def make_records(slope=0.002, intercept=0.02, n=6, t0=10, dt=10,
                 speed=2000.0, feed=300.0, depth=2.0):
    """构造理想线性磨损记录：t=10..60，wear=0.02+0.002t，限值 0.3 时寿命 140min。"""
    recs = []
    for i in range(n):
        t = t0 + i * dt
        recs.append({
            "cutting_time_min": float(t),
            "wear_mm": round(intercept + slope * t, 6),
            "speed_rpm": speed, "feed_mm_min": feed, "depth_mm": depth,
        })
    return recs


# ---------------------------------------------------------------------- #
# 数据校验
# ---------------------------------------------------------------------- #
def test_validate_records_rejects_empty():
    clean, errors, _ = validate_records([])
    assert clean == [] and errors


def test_validate_records_rejects_bad_values():
    _, errors, _ = validate_records([
        {"cutting_time_min": -5, "wear_mm": 0.1},
        {"cutting_time_min": 10, "wear_mm": -0.1},
        {"cutting_time_min": 20, "wear_mm": 0.1, "speed_rpm": "abc"},
        "not-a-dict",
    ])
    assert len(errors) == 4


def test_validate_records_sorts_and_flags_wear_decrease():
    clean, errors, anomalies = validate_records([
        {"cutting_time_min": 30, "wear_mm": 0.05},   # 磨损回退
        {"cutting_time_min": 10, "wear_mm": 0.10},
        {"cutting_time_min": 20, "wear_mm": 0.12},
    ])
    assert errors == []
    assert [r["cutting_time_min"] for r in clean] == [10, 20, 30]
    assert any("不可回退" in a and "测量误差" in a for a in anomalies)


def test_predict_requires_two_records():
    with pytest.raises(LifeValidationError) as exc:
        predict_tool_life([{"cutting_time_min": 10, "wear_mm": 0.1}])
    assert exc.value.errors


def test_predict_rejects_bad_life_config():
    with pytest.raises(LifeValidationError) as exc:
        predict_tool_life(make_records(),
                          life_config={"wear_limit_mm": -1,
                                       "confidence_level": 2})
    assert any("wear_limit_mm" in e for e in exc.value.errors)
    assert any("confidence_level" in e for e in exc.value.errors)


def test_predict_rejects_bad_condition():
    with pytest.raises(LifeValidationError) as exc:
        predict_tool_life(make_records(), condition={"speed_rpm": 0})
    assert any("speed_rpm" in e for e in exc.value.errors)


# ---------------------------------------------------------------------- #
# 趋势与寿命估算
# ---------------------------------------------------------------------- #
def test_predict_linear_trend_exact():
    rep = predict_tool_life(make_records(),
                            condition={"speed_rpm": 2000, "feed_mm_min": 300,
                                       "depth_mm": 2.0})
    wear = rep["wear"]
    assert wear["wear_rate_mm_per_min"] == pytest.approx(0.002, abs=1e-6)
    assert wear["r_squared"] == pytest.approx(1.0)
    assert wear["current_wear_mm"] == pytest.approx(0.14)
    pred = rep["prediction"]
    # 拟合线 0.02+0.002t 触及 0.3 于 t=140；已切削 60min → 剩余 80min
    assert pred["remaining_base_min"] == pytest.approx(80.0, abs=1e-3)
    assert pred["condition_factor"] == pytest.approx(1.0)
    assert pred["remaining_min"] == pytest.approx(80.0, abs=1e-3)
    assert pred["total_life_min"] == pytest.approx(140.0, abs=1e-3)
    lo, hi = pred["remaining_ci_min"]
    assert lo <= pred["remaining_min"] <= hi
    assert rep["threshold_violations"] == []
    assert rep["anomalies"] == []


def test_confidence_interval_widens_with_noise():
    clean_rep = predict_tool_life(make_records())
    noisy = make_records()
    for i, r in enumerate(noisy):
        r["wear_mm"] += 0.01 if i % 2 else -0.01
    noisy_rep = predict_tool_life(noisy)
    clean_width = (clean_rep["prediction"]["remaining_ci_min"][1]
                   - clean_rep["prediction"]["remaining_ci_min"][0])
    noisy_width = (noisy_rep["prediction"]["remaining_ci_min"][1]
                   - noisy_rep["prediction"]["remaining_ci_min"][0])
    assert noisy_width > clean_width
    assert noisy_rep["wear"]["r_squared"] < 1.0


def test_speed_is_dominant_factor():
    """转速翻倍（指数 4）应主导寿命缩短：factor=0.5^4=0.0625。"""
    rep = predict_tool_life(make_records(),
                            condition={"speed_rpm": 4000, "feed_mm_min": 300,
                                       "depth_mm": 2.0})
    pred = rep["prediction"]
    assert pred["condition_factor"] == pytest.approx(0.0625)
    assert pred["remaining_min"] == pytest.approx(80.0 * 0.0625, abs=1e-3)
    top = rep["factors"][0]
    assert top["parameter"] == "speed_rpm"
    assert top["share"] == pytest.approx(1.0)
    assert "缩短" in top["note"]
    # 剩余寿命 5min 低于默认 30min 安全余量 → 提示换刀
    assert any("换刀" in a for a in rep["anomalies"])


def test_milder_condition_extends_life():
    rep = predict_tool_life(make_records(),
                            condition={"speed_rpm": 1500, "feed_mm_min": 300,
                                       "depth_mm": 2.0})
    assert rep["prediction"]["condition_factor"] > 1.0
    assert (rep["prediction"]["remaining_min"]
            > rep["prediction"]["remaining_base_min"])


def test_threshold_violation_reported():
    rep = predict_tool_life(
        make_records(),
        condition={"speed_rpm": 9000, "feed_mm_min": 300, "depth_mm": 2.0},
        life_config={"max_speed_rpm": 8000, "max_depth_mm": 3.0})
    assert len(rep["threshold_violations"]) == 1
    v = rep["threshold_violations"][0]
    assert v["parameter"] == "speed_rpm"
    assert v["value"] == 9000 and v["limit"] == 8000
    assert "超出安全阈值" in v["message"]
    assert "超出安全阈值" in rep["verdict"]


def test_wear_already_over_limit():
    recs = make_records(slope=0.01)  # 末点 wear=0.62 > 0.3
    rep = predict_tool_life(recs)
    assert rep["prediction"]["remaining_min"] == 0.0
    assert any("磨钝标准" in a for a in rep["anomalies"])
    assert "立即换刀" in rep["verdict"]


def test_flat_wear_cannot_extrapolate():
    recs = [{"cutting_time_min": float(t), "wear_mm": 0.1}
            for t in (10, 20, 30, 40)]
    rep = predict_tool_life(recs)
    assert rep["prediction"]["remaining_min"] is None
    assert rep["prediction"]["remaining_ci_min"] is None
    assert any("非增长" in a for a in rep["anomalies"])


def test_low_r_squared_flagged():
    recs = [
        {"cutting_time_min": 10, "wear_mm": 0.02},
        {"cutting_time_min": 20, "wear_mm": 0.12},
        {"cutting_time_min": 30, "wear_mm": 0.03},
        {"cutting_time_min": 40, "wear_mm": 0.14},
        {"cutting_time_min": 50, "wear_mm": 0.05},
    ]
    rep = predict_tool_life(recs)
    assert rep["wear"]["r_squared"] < 0.6
    assert any("R²" in a for a in rep["anomalies"])


def test_material_mismatch_flagged():
    rep = predict_tool_life(make_records(), material="45钢",
                            condition={"material": "铝合金"})
    assert any("材料" in a and "仅供参考" in a for a in rep["anomalies"])
    # 显式材料系数则直接生效、不再提示
    rep2 = predict_tool_life(make_records(), material="45钢",
                             condition={"material": "铝合金",
                                        "material_factor": 2.0})
    assert rep2["prediction"]["condition_factor"] == pytest.approx(2.0)
    assert not any("材料" in a for a in rep2["anomalies"])


# ---------------------------------------------------------------------- #
# API
# ---------------------------------------------------------------------- #
@pytest.fixture
def client(tmp_path):
    app = create_app(str(tmp_path / "tool_life.db"))
    app.config["TESTING"] = True
    return app.test_client()


def _import_one(client, tool_id="T1", material="45钢", records=None):
    return client.post("/api/tool-life/records", json={
        "tools": [{"tool_id": tool_id, "material": material,
                   "records": records or make_records()}]})


def test_batch_import_and_query(client):
    resp = _import_one(client)
    assert resp.status_code == 201
    assert resp.get_json()["imported"] == [{"tool_id": "T1", "records": 6}]

    tools = client.get("/api/tool-life/records").get_json()
    assert tools[0]["tool_id"] == "T1"
    assert tools[0]["record_count"] == 6
    assert tools[0]["material"] == "45钢"

    detail = client.get("/api/tool-life/records/T1").get_json()
    assert detail["record_count"] == 6
    times = [r["cutting_time_min"] for r in detail["records"]]
    assert times == sorted(times)

    assert client.get("/api/tool-life/records/UNKNOWN").status_code == 404


def test_batch_import_partial_failure(client):
    resp = client.post("/api/tool-life/records", json={"tools": [
        {"tool_id": "T1", "records": make_records()},
        {"tool_id": "T2", "records": [{"cutting_time_min": -1, "wear_mm": 0.1}]},
        {"records": []},
    ]})
    body = resp.get_json()
    assert resp.status_code == 201
    assert len(body["imported"]) == 1
    assert len(body["failed"]) == 2
    assert body["failed"][0]["tool_id"] == "T2"
    assert body["failed"][0]["errors"]


def test_batch_import_all_failed_returns_400(client):
    resp = client.post("/api/tool-life/records",
                       json={"tools": [{"tool_id": "T2", "records": "bad"}]})
    assert resp.status_code == 400
    assert resp.get_json()["failed"]


def test_import_upsert_same_timestamp(client):
    _import_one(client)
    resp = client.post("/api/tool-life/records", json={
        "tool_id": "T1",
        "records": [{"cutting_time_min": 60, "wear_mm": 0.20},
                    {"cutting_time_min": 70, "wear_mm": 0.22}]})
    assert resp.status_code == 201
    detail = client.get("/api/tool-life/records/T1").get_json()
    assert detail["record_count"] == 7          # t=60 被覆盖而非新增
    last = {r["cutting_time_min"]: r["wear_mm"] for r in detail["records"]}
    assert last[60.0] == 0.20


def test_predict_inline(client):
    resp = client.post("/api/tool-life/predict", json={
        "tool_id": "T9", "records": make_records(),
        "condition": {"speed_rpm": 2000, "feed_mm_min": 300, "depth_mm": 2.0},
    })
    assert resp.status_code == 200
    body = resp.get_json()
    assert body["tool_id"] == "T9"
    assert body["prediction"]["remaining_min"] == pytest.approx(80.0, abs=1e-3)
    assert body["prediction"]["remaining_ci_min"][0] <= 80.0
    assert body["factors"] and body["verdict"]


def test_predict_inline_validation_error_has_details(client):
    resp = client.post("/api/tool-life/predict",
                       json={"records": [{"cutting_time_min": -1}]})
    assert resp.status_code == 400
    body = resp.get_json()
    assert body["error"].startswith("刀具寿命数据校验失败")
    assert body["details"]


def test_predict_saved_tool_and_export(client):
    _import_one(client)
    resp = client.post("/api/tool-life/predict/T1", json={
        "condition": {"speed_rpm": 2500, "feed_mm_min": 300, "depth_mm": 2.0},
        "life_config": {"max_speed_rpm": 2400},
    })
    assert resp.status_code == 200
    body = resp.get_json()
    assert body["material"] == "45钢"
    assert body["prediction"]["condition_factor"] == pytest.approx(0.8 ** 4)
    assert body["threshold_violations"][0]["parameter"] == "speed_rpm"

    # JSON 导出：?download=1 返回附件
    dl = client.post("/api/tool-life/predict/T1?download=1",
                     json={"condition": {"speed_rpm": 2000}})
    assert dl.status_code == 200
    assert "attachment" in dl.headers["Content-Disposition"]
    assert "tool_life_T1.json" in dl.headers["Content-Disposition"]
    exported = json.loads(dl.data.decode("utf-8"))
    assert exported["prediction"]["remaining_min"] == pytest.approx(80.0,
                                                                    abs=1e-3)


def test_predict_saved_tool_missing(client):
    assert client.post("/api/tool-life/predict/T404",
                       json={"condition": {}}).status_code == 404


def test_predict_inline_reuses_stored_records(client):
    _import_one(client)
    resp = client.post("/api/tool-life/predict",
                       json={"tool_id": "T1", "condition": {}})
    assert resp.status_code == 200
    assert resp.get_json()["record_count"] == 6
