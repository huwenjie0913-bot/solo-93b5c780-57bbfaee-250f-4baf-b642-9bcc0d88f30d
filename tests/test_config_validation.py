"""配置入口校验缺陷回归测试。

1. tools/h_offsets/d_offsets/envelope/work_offsets 不是对象时，
   validate_config 与 normalize_config 不得抛 AttributeError，应给出结构化错误；
2. NaN/无穷的尺寸、坐标与安全参数一律拒绝（此前会漏过校验，
   重建出 nan 轨迹并把程序误报为 clean）；
3. Flask 路由对校验失败返回 400 + {"error", "details"} 结构化响应。
"""

import json
import math

import pytest

from gcode_review.app import create_app
from gcode_review.config import (
    DEFAULT_CONFIG,
    ConfigValidationError,
    normalize_config,
    validate_config,
)

NAN = float("nan")
INF = float("inf")

VALID_PROGRAM = "G21 G90 G54\nT1 M06\nG00 G43 H1 Z50.\nM30\n"


def _valid_config():
    return {
        "envelope": {"xmin": 0.0, "xmax": 500.0,
                     "ymin": 0.0, "ymax": 400.0,
                     "zmin": 0.0, "zmax": 450.0},
        "work_offsets": {f"G5{n}": {"x": 0.0, "y": 0.0, "z": 0.0}
                         for n in range(4, 10)},
        "safety_clearance": 50.0,
        "tools": {"1": {"length": 100.0, "diameter": 10.0}},
    }


def _errors(cfg):
    _, errors = validate_config(cfg)
    return errors


# --------------------------------------------------------------------- #
# 非对象配置节：结构化错误而非 AttributeError
# --------------------------------------------------------------------- #
@pytest.mark.parametrize("section", ["tools", "h_offsets", "d_offsets",
                                     "envelope", "work_offsets"])
@pytest.mark.parametrize("bad", [["not", "a", "dict"], "garbage", 42, 3.5, True])
def test_non_object_section_returns_structured_error(section, bad):
    cfg = _valid_config()
    cfg[section] = bad
    errors = _errors(cfg)  # 不得抛 AttributeError
    assert any(section in e and "对象" in e for e in errors), errors


@pytest.mark.parametrize("section", ["tools", "h_offsets", "d_offsets",
                                     "envelope", "work_offsets"])
def test_normalize_config_tolerates_non_object_sections(section):
    cfg = _valid_config()
    cfg[section] = ["not", "a", "dict"]
    normalized = normalize_config(cfg)  # 不得抛 AttributeError
    assert isinstance(normalized[section], dict)


def test_validate_config_rejects_non_dict_raw():
    _, errors = validate_config(["not", "a", "dict"])
    assert errors and "对象" in errors[0]
    _, errors_none = validate_config(None)
    assert errors_none


def test_tool_entry_must_be_object():
    cfg = _valid_config()
    cfg["tools"] = {"1": "not-an-object", "2": 42}
    errors = _errors(cfg)
    assert any("tools.1" in e and "对象" in e for e in errors)
    assert any("tools.2" in e and "对象" in e for e in errors)


# --------------------------------------------------------------------- #
# NaN / 无穷：尺寸、坐标、安全参数一律拒绝
# --------------------------------------------------------------------- #
@pytest.mark.parametrize("value", [NAN, INF, -INF])
def test_envelope_rejects_non_finite(value):
    cfg = _valid_config()
    cfg["envelope"]["xmin"] = value
    errors = _errors(cfg)
    assert any("envelope.xmin" in e for e in errors), errors


@pytest.mark.parametrize("value", [NAN, INF, -INF])
def test_safety_clearance_rejects_non_finite(value):
    cfg = _valid_config()
    cfg["safety_clearance"] = value
    assert any("safety_clearance" in e for e in _errors(cfg))


@pytest.mark.parametrize("value", [NAN, INF, -INF])
def test_work_offset_rejects_non_finite(value):
    cfg = _valid_config()
    cfg["work_offsets"]["G54"]["z"] = value
    assert any("work_offsets.G54" in e for e in _errors(cfg))


@pytest.mark.parametrize("value", [NAN, INF, -INF, "abc", None])
def test_h_offsets_reject_non_finite_or_text(value):
    cfg = _valid_config()
    cfg["h_offsets"] = {"1": value}
    assert any("h_offsets.1" in e for e in _errors(cfg))


@pytest.mark.parametrize("value", [NAN, INF, "abc"])
def test_d_offsets_reject_non_finite_or_text(value):
    cfg = _valid_config()
    cfg["d_offsets"] = {"2": value}
    assert any("d_offsets.2" in e for e in _errors(cfg))


@pytest.mark.parametrize("field", ["rapid_speed", "default_feed", "default_spindle",
                                   "tool_change_time", "arc_tolerance"])
@pytest.mark.parametrize("value", [NAN, INF, -INF])
def test_speed_params_reject_non_finite(field, value):
    cfg = _valid_config()
    cfg[field] = value
    assert any(field in e for e in _errors(cfg))


@pytest.mark.parametrize("field", ["length", "diameter", "flute_length",
                                   "shank_diameter", "holder_length"])
@pytest.mark.parametrize("value", [NAN, INF])
def test_tool_dimensions_reject_non_finite(field, value):
    cfg = _valid_config()
    cfg["tools"]["1"][field] = value
    assert any(f"tools.1.{field}" in e for e in _errors(cfg))


@pytest.mark.parametrize("value", [NAN, INF])
def test_obstacle_coordinate_rejects_non_finite(value):
    cfg = _valid_config()
    cfg["obstacles"] = [{"id": "v", "type": "box",
                         "min": {"x": value, "y": 0, "z": 0},
                         "max": {"x": 1, "y": 1, "z": 1}}]
    assert any("obstacles[0].min.x" in e for e in _errors(cfg))


@pytest.mark.parametrize("value", [NAN, INF])
def test_collision_params_reject_non_finite(value):
    cfg = _valid_config()
    cfg["collision_max_step"] = value
    assert any("collision_max_step" in e for e in _errors(cfg))
    cfg2 = _valid_config()
    cfg2["collision_clearance_warn"] = value
    assert any("collision_clearance_warn" in e for e in _errors(cfg2))


def test_start_position_rejects_non_object_and_non_finite():
    cfg = _valid_config()
    cfg["start_position"] = "garbage"
    assert any("start_position" in e and "对象" in e for e in _errors(cfg))
    cfg2 = _valid_config()
    cfg2["start_position"] = {"x": NAN, "y": 0.0}
    assert any("start_position.x" in e for e in _errors(cfg2))


def test_valid_config_still_passes():
    assert _errors(_valid_config()) == []
    assert _errors({}) == []  # 全默认配置同样有效


def test_config_validation_error_carries_details():
    exc = ConfigValidationError(["甲", "乙"])
    assert exc.errors == ["甲", "乙"]
    assert "甲" in str(exc) and "乙" in str(exc)
    assert isinstance(exc, ValueError)


# --------------------------------------------------------------------- #
# Flask 路由：400 + 结构化 details
# --------------------------------------------------------------------- #
@pytest.fixture
def client(tmp_path):
    app = create_app(str(tmp_path / "test.db"))
    return app.test_client()


def _post_configs(client, config, name="m1"):
    return client.post("/api/configs", json={"name": name, "config": config})


def test_post_configs_non_object_sections_400(client):
    resp = _post_configs(client, {**_valid_config(), "tools": ["not", "a", "dict"]})
    assert resp.status_code == 400
    payload = resp.get_json()
    assert "error" in payload
    assert isinstance(payload["details"], list)
    assert any("tools" in d and "对象" in d for d in payload["details"])


def test_post_configs_nan_envelope_400(client):
    cfg = _valid_config()
    cfg["envelope"]["xmax"] = NAN
    resp = _post_configs(client, cfg)
    assert resp.status_code == 400
    assert any("envelope.xmax" in d for d in resp.get_json()["details"])


def test_post_configs_inf_safety_clearance_400(client):
    resp = _post_configs(client, {**_valid_config(), "safety_clearance": INF})
    assert resp.status_code == 400
    assert any("safety_clearance" in d for d in resp.get_json()["details"])


def test_post_configs_valid_201_and_roundtrip(client):
    resp = _post_configs(client, _valid_config())
    assert resp.status_code == 201
    cid = resp.get_json()["id"]
    got = client.get(f"/api/configs/{cid}")
    assert got.status_code == 200
    assert got.get_json()["config"]["tools"]["1"]["length"] == 100.0


def test_review_inline_non_object_work_offsets_400(client):
    resp = client.post("/api/review", json={
        "content": VALID_PROGRAM,
        "config": {**_valid_config(), "work_offsets": ["oops"]}})
    assert resp.status_code == 400
    assert any("work_offsets" in d for d in resp.get_json()["details"])


def test_review_inline_nan_h_offset_400(client):
    """回归：NaN 刀长补偿此前会漏过校验，重建出 nan 轨迹并误报 clean。"""
    resp = client.post("/api/review", json={
        "content": VALID_PROGRAM,
        "config": {**_valid_config(), "h_offsets": {"1": NAN}}})
    assert resp.status_code == 400
    assert any("h_offsets.1" in d for d in resp.get_json()["details"])


def test_review_inline_nan_safety_clearance_400(client):
    resp = client.post("/api/review", json={
        "content": VALID_PROGRAM,
        "config": {**_valid_config(), "safety_clearance": NAN}})
    assert resp.status_code == 400
    assert any("safety_clearance" in d for d in resp.get_json()["details"])


def test_review_inline_config_must_be_object(client):
    resp = client.post("/api/review", json={
        "content": VALID_PROGRAM, "config": ["not", "a", "dict"]})
    assert resp.status_code == 400
    assert "对象" in resp.get_json()["error"]


def test_review_saved_program_rejects_bad_config(client):
    pid = client.post("/api/programs", json={
        "name": "p.nc", "content": VALID_PROGRAM}).get_json()["id"]
    resp = client.post(f"/api/review/program/{pid}", json={
        "config": {**_valid_config(), "envelope": "garbage"}})
    assert resp.status_code == 400
    assert any("envelope" in d and "对象" in d
               for d in resp.get_json()["details"])


def test_compare_rejects_bad_config(client):
    resp = client.post("/api/compare", json={
        "content": VALID_PROGRAM,
        "config_a": {**_valid_config(), "d_offsets": {"1": INF}},
        "config_b": _valid_config()})
    assert resp.status_code == 400
    assert any("d_offsets.1" in d for d in resp.get_json()["details"])


def test_non_object_json_body_400(client):
    for path in ("/api/review", "/api/compare", "/api/configs", "/api/programs"):
        resp = client.post(path, data="[1, 2, 3]",
                           content_type="application/json")
        assert resp.status_code == 400, path
        assert "error" in resp.get_json()


def test_review_valid_config_200_and_no_nan_in_response(client):
    resp = client.post("/api/review", json={
        "content": VALID_PROGRAM, "config": _valid_config()})
    assert resp.status_code == 200
    # 响应必须是严格 JSON：不得含 NaN/Infinity 常量
    json.loads(resp.get_data(as_text=True),
               parse_constant=_reject_constant)


def test_review_default_config_no_nan_in_response(client):
    resp = client.post("/api/review", json={"content": VALID_PROGRAM})
    assert resp.status_code == 200
    json.loads(resp.get_data(as_text=True),
               parse_constant=_reject_constant)


def _reject_constant(value):
    raise AssertionError(f"响应含非标准 JSON 常量 {value}（NaN/Infinity）")
