"""Flask 应用：G-code 静态审查 HTTP API。

路由一览：
* GET  /health
* POST /api/programs                 上传程序（multipart 文件或 JSON {name,content}）
* GET  /api/programs                 程序列表
* GET  /api/programs/<id>            程序详情
* POST /api/configs                  保存机床配置（自动递增版本）
* GET  /api/configs                  配置版本列表
* GET  /api/configs/<id>             配置详情
* POST /api/review                   即时审查（程序 id 或内联文本 + 配置）
* POST /api/review/program/<pid>     已存程序审查（配置 id/name+version 或内联）
* POST /api/compare                  同一程序两套配置的风险差异
* POST /api/resume                   断点续跑审查（重放前缀 + 生成恢复前导段 + 复核）
* POST /api/tool-life/records        批量导入刀具磨损记录
* GET  /api/tool-life/records        刀具记录汇总列表
* GET  /api/tool-life/records/<tid>  某刀具磨损记录详情
* POST /api/tool-life/predict        即时寿命预测（内联记录或 tool_id + 本次工况）
* POST /api/tool-life/predict/<tid>  已存刀具寿命预测（?download=1 导出 JSON）
"""

from __future__ import annotations

import hashlib
import json
import math
import os
from typing import Any, Dict, Tuple

from flask import Flask, jsonify, request

from .compare import compare_reviews
from .config import (DEFAULT_CONFIG, ConfigValidationError, normalize_config,
                     validate_config)
from .db import Database
from .engine import review_program
from .resume import ResumeValidationError, review_resume
from .tool_life import LifeValidationError, predict_tool_life, validate_records


def create_app(db_path: str = None) -> Flask:
    app = Flask(__name__)
    db_path = db_path or os.environ.get(
        "GCODE_DB_PATH", os.path.join(os.getcwd(), "gcode_review.db"))
    db = Database(db_path)
    app.config["DB"] = db

    @app.teardown_appcontext
    def _close(_exc):  # 单连接常驻，进程结束随库文件；保留钩子便于扩展
        return None

    def err(status: int, message: str, **extra):
        payload = {"error": message}
        payload.update(extra)
        return jsonify(payload), status

    def json_object():
        """解析请求 JSON 对象体。无 JSON 体返回 {}；JSON 不是对象返回 None。"""
        body = request.get_json(silent=True)
        if body is None:
            return {}
        return body if isinstance(body, dict) else None

    def config_exc_response(e: Exception):
        """配置引用/校验异常的统一响应：校验错误带结构化 details 列表。"""
        if isinstance(e, ConfigValidationError):
            return err(400, str(e), details=e.errors)
        return err(400 if isinstance(e, ValueError) else 404, str(e))

    def get_program_content(payload, files) -> Tuple[str, str]:
        """返回 (name, content)。"""
        if files and "file" in files:
            f = files["file"]
            raw = f.read()
            try:
                content = raw.decode("utf-8")
            except UnicodeDecodeError:
                content = raw.decode("latin-1")
            return f.filename, content
        name = (payload.get("name") or "inline.gcode")
        content = payload.get("content")
        if not isinstance(content, str):
            raise ValueError("需要 content（G-code 文本）或 multipart file 字段")
        return name, content

    def resolve_config(body) -> Dict[str, Any]:
        """从请求体解析配置：config_id，或 config_name(+config_version)，或 config 内联。"""
        if body.get("config_id") is not None:
            row = db.get_config(int(body["config_id"]))
            if not row:
                raise LookupError(f"config_id={body['config_id']} 不存在")
            return row["config"]
        if body.get("config_name"):
            row = db.resolve_config_ref(body["config_name"],
                                       body.get("config_version"))
            if not row:
                raise LookupError(
                    f"机床配置 {body['config_name']} "
                    f"v{body.get('config_version', 'latest')} 不存在")
            return row["config"]
        if "config" in body:
            if not isinstance(body["config"], dict):
                raise ValueError("config 必须是对象（JSON object）")
            cfg, errors = validate_config(body["config"])
            if errors:
                raise ConfigValidationError(errors)
            return cfg
        # 未给配置：用默认配置
        return normalize_config(DEFAULT_CONFIG)

    # ------------------------------------------------------------------ #
    @app.get("/health")
    def health():
        return jsonify({"status": "ok", "service": "gcode-static-review"})

    # ---------------- 程序 --------------------------------------------- #
    @app.post("/api/programs")
    def upload_program():
        payload = request.form
        if not payload:
            payload = json_object()
            if payload is None:
                return err(400, "请求体必须是 JSON 对象")
        try:
            name, content = get_program_content(payload, request.files)
        except ValueError as e:
            return err(400, str(e))
        if not content.strip():
            return err(400, "程序内容为空")
        sha = hashlib.sha256(content.encode("utf-8")).hexdigest()[:16]
        pid = db.insert_program(name, content, sha)
        return jsonify({"id": pid, "name": name, "sha256": sha}), 201

    @app.get("/api/programs")
    def list_programs():
        return jsonify(db.list_programs())

    @app.get("/api/programs/<int:pid>")
    def get_program(pid):
        row = db.get_program(pid)
        if not row:
            return err(404, f"程序 {pid} 不存在")
        return jsonify(row)

    # ---------------- 配置 --------------------------------------------- #
    @app.post("/api/configs")
    def create_config():
        body = json_object()
        if body is None:
            return err(400, "请求体必须是 JSON 对象")
        name = body.get("name")
        if not name or not isinstance(name, str):
            return err(400, "需要 name（配置名称）")
        raw = body.get("config")
        if not isinstance(raw, dict):
            return err(400, "需要 config（机床配置对象）")
        try:
            cid, version = db.save_config(name, raw, note=body.get("note", ""))
        except ConfigValidationError as e:
            return err(400, str(e), details=e.errors)
        except ValueError as e:
            return err(400, str(e))
        return jsonify({"id": cid, "name": name, "version": version}), 201

    @app.get("/api/configs")
    def list_configs():
        return jsonify(db.list_configs())

    @app.get("/api/configs/<int:cid>")
    def get_config(cid):
        row = db.get_config(cid)
        if not row:
            return err(404, f"配置 {cid} 不存在")
        return jsonify(row)

    # ---------------- 审查 --------------------------------------------- #
    @app.post("/api/review")
    def review_inline():
        body = json_object()
        if body is None:
            return err(400, "请求体必须是 JSON 对象")
        content = body.get("content")
        name = body.get("name", "inline.gcode")
        pid = body.get("program_id")
        if pid is not None:
            row = db.get_program(int(pid))
            if not row:
                return err(404, f"程序 {pid} 不存在")
            content, name = row["content"], row["name"]
        if not isinstance(content, str) or not content.strip():
            return err(400, "需要 content（G-code 文本）或 program_id")
        try:
            cfg = resolve_config(body)
        except (ValueError, LookupError) as e:
            return config_exc_response(e)
        return jsonify(review_program(content, cfg, filename=name))

    @app.post("/api/review/program/<int:pid>")
    def review_saved(pid):
        row = db.get_program(pid)
        if not row:
            return err(404, f"程序 {pid} 不存在")
        body = json_object()
        if body is None:
            return err(400, "请求体必须是 JSON 对象")
        try:
            cfg = resolve_config(body)
        except (ValueError, LookupError) as e:
            return config_exc_response(e)
        return jsonify(review_program(row["content"], cfg, filename=row["name"]))

    # ---------------- 刀具寿命 ----------------------------------------- #
    def maybe_download(report: Dict[str, Any], filename: str):
        """?download=1 时以附件形式导出 JSON 结果，否则正常 JSON 响应。"""
        if request.args.get("download"):
            resp = app.response_class(
                json.dumps(report, ensure_ascii=False, indent=2),
                mimetype="application/json")
            resp.headers["Content-Disposition"] = \
                f"attachment; filename={filename}"
            return resp
        return jsonify(report)

    @app.post("/api/tool-life/records")
    def import_tool_life_records():
        body = json_object()
        if body is None:
            return err(400, "请求体必须是 JSON 对象")
        tools = body.get("tools")
        if tools is None:
            # 兼容单刀具直接提交：{tool_id, material, records}
            if body.get("tool_id") is not None:
                tools = [body]
            else:
                return err(400, "需要 tools（刀具记录数组）或单条 tool_id+records")
        if not isinstance(tools, list) or not tools:
            return err(400, "tools 必须是非空数组")
        imported, failed = [], []
        for i, item in enumerate(tools):
            if not isinstance(item, dict):
                failed.append({"index": i, "errors": ["必须是对象（JSON object）"]})
                continue
            tool_id = item.get("tool_id")
            if not tool_id or not isinstance(tool_id, str):
                failed.append({"index": i, "tool_id": tool_id,
                               "errors": ["需要 tool_id（刀具标识字符串）"]})
                continue
            records, rec_errors, _anomalies = validate_records(
                item.get("records"), min_count=1)
            if rec_errors:
                failed.append({"index": i, "tool_id": tool_id,
                               "errors": rec_errors})
                continue
            n = db.upsert_tool_life_records(
                tool_id, str(item.get("material", "")), records)
            imported.append({"tool_id": tool_id, "records": n})
        payload: Dict[str, Any] = {"imported": imported, "failed": failed}
        if not imported:
            payload["error"] = "没有可导入的刀具记录"
            return jsonify(payload), 400
        return jsonify(payload), 201

    @app.get("/api/tool-life/records")
    def list_tool_life_records():
        return jsonify(db.list_tool_life_tools())

    @app.get("/api/tool-life/records/<tool_id>")
    def get_tool_life_records(tool_id):
        row = db.get_tool_life_records(tool_id)
        if not row:
            return err(404, f"刀具 {tool_id} 无磨损记录")
        return jsonify(row)

    @app.post("/api/tool-life/predict")
    def tool_life_predict_inline():
        body = json_object()
        if body is None:
            return err(400, "请求体必须是 JSON 对象")
        records = body.get("records")
        tool_id = body.get("tool_id")
        material = body.get("material")
        if records is None and tool_id is not None:
            stored = db.get_tool_life_records(str(tool_id))
            if not stored:
                return err(404, f"刀具 {tool_id} 无磨损记录，且请求未提供 records")
            records = stored["records"]
            material = material or stored.get("material")
        if records is None:
            return err(400, "需要 records（磨损记录数组）或已存刀具的 tool_id")
        try:
            report = predict_tool_life(
                records, condition=body.get("condition"),
                life_config=body.get("life_config"),
                tool_id=tool_id, material=material)
        except LifeValidationError as e:
            return err(400, str(e), details=e.errors)
        return maybe_download(report, f"tool_life_{tool_id or 'inline'}.json")

    @app.post("/api/tool-life/predict/<tool_id>")
    def tool_life_predict_saved(tool_id):
        stored = db.get_tool_life_records(tool_id)
        if not stored:
            return err(404, f"刀具 {tool_id} 无磨损记录")
        body = json_object()
        if body is None:
            return err(400, "请求体必须是 JSON 对象")
        try:
            report = predict_tool_life(
                stored["records"], condition=body.get("condition"),
                life_config=body.get("life_config"),
                tool_id=tool_id, material=stored.get("material"))
        except LifeValidationError as e:
            return err(400, str(e), details=e.errors)
        return maybe_download(report, f"tool_life_{tool_id}.json")

    # ---------------- 比对 --------------------------------------------- #
    @app.post("/api/compare")
    def compare():
        body = json_object()
        if body is None:
            return err(400, "请求体必须是 JSON 对象")
        pid = body.get("program_id")
        content = body.get("content")
        name = body.get("name", "inline.gcode")
        if pid is not None:
            row = db.get_program(int(pid))
            if not row:
                return err(404, f"程序 {pid} 不存在")
            content, name = row["content"], row["name"]
        if not isinstance(content, str) or not content.strip():
            return err(400, "需要 content 或 program_id")

        def one_cfg(spec):
            if isinstance(spec, dict) and spec.get("config_id") is not None:
                r = db.get_config(int(spec["config_id"]))
                if not r:
                    raise LookupError(f"config_id={spec['config_id']} 不存在")
                return r["config"]
            if isinstance(spec, dict) and spec.get("config_name"):
                r = db.resolve_config_ref(spec["config_name"],
                                          spec.get("config_version"))
                if not r:
                    raise LookupError(f"配置 {spec['config_name']} 不存在")
                return r["config"]
            if isinstance(spec, dict):
                cfg, errors = validate_config(spec)
                if errors:
                    raise ConfigValidationError(errors)
                return cfg
            raise ValueError("config_a/config_b 必须是配置对象或配置引用")

        try:
            cfg_a = one_cfg(body.get("config_a"))
            cfg_b = one_cfg(body.get("config_b"))
        except (ValueError, LookupError) as e:
            return config_exc_response(e)
        return jsonify(compare_reviews(content, cfg_a, cfg_b, filename=name))

    # ---------------- 断点续跑 ----------------------------------------- #
    @app.post("/api/resume")
    def resume_review():
        body = json_object()
        if body is None:
            return err(400, "请求体必须是 JSON 对象")
        pid = body.get("program_id")
        content = body.get("content")
        name = body.get("name", "inline.gcode")
        if pid is not None:
            row = db.get_program(int(pid))
            if not row:
                return err(404, f"程序 {pid} 不存在")
            content, name = row["content"], row["name"]
        if not isinstance(content, str) or not content.strip():
            return err(400, "需要 content（G-code 文本）或 program_id")
        try:
            cfg = resolve_config(body)
        except (ValueError, LookupError) as e:
            return config_exc_response(e)

        resume_line = body.get("resume_line")
        if not isinstance(resume_line, int) or isinstance(resume_line, bool) \
                or resume_line < 1:
            return err(400, "需要 resume_line（计划恢复执行的物理行号，从 1 开始的正整数）")

        measured = body.get("measured_position")
        if measured is not None and not isinstance(measured, dict):
            return err(400, "measured_position 必须是含 x/y/z 机床坐标的对象")
        tol = body.get("position_tolerance", 0.01)
        if not isinstance(tol, (int, float)) or isinstance(tol, bool) \
                or not math.isfinite(float(tol)) or tol < 0:
            return err(400, "position_tolerance 必须是非负有限数值（mm）")

        try:
            report = review_resume(
                content, cfg, resume_line=resume_line,
                measured_position=measured,
                position_tolerance=float(tol), filename=name)
        except ResumeValidationError as e:
            return err(400, str(e))
        return jsonify(report)

    @app.errorhandler(404)
    def _404(_e):
        return jsonify({"error": "路径不存在"}), 404

    @app.errorhandler(405)
    def _405(_e):
        return jsonify({"error": "方法不允许"}), 405

    return app


def main() -> None:
    """控制台入口：python -m gcode_review / 安装后的 gcode-review 命令。"""
    import argparse

    parser = argparse.ArgumentParser(description="G-code 静态审查 API 服务")
    parser.add_argument("--host", default=os.environ.get("HOST", "0.0.0.0"))
    parser.add_argument("--port", type=int,
                        default=int(os.environ.get("PORT", 5000)))
    parser.add_argument("--db", default=os.environ.get("GCODE_DB_PATH"),
                        help="SQLite 库路径（默认 ./gcode_review.db）")
    args = parser.parse_args()

    app = create_app(args.db)
    app.run(host=args.host, port=args.port)


if __name__ == "__main__":
    main()
