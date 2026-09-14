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
"""

from __future__ import annotations

import hashlib
import os
from typing import Any, Dict, Tuple

from flask import Flask, jsonify, request

from .compare import compare_reviews
from .config import (DEFAULT_CONFIG, ConfigValidationError, normalize_config,
                     validate_config)
from .db import Database
from .engine import review_program


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
