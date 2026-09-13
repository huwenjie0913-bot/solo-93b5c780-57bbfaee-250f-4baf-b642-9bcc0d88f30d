"""SQLite 持久化：程序、版本化机床配置。仅使用标准库 sqlite3。"""

from __future__ import annotations

import json
import os
import sqlite3
import time
from typing import Any, Dict, List, Optional, Tuple

from .config import validate_config

SCHEMA = """
CREATE TABLE IF NOT EXISTS programs (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    name        TEXT NOT NULL,
    content     TEXT NOT NULL,
    sha256      TEXT NOT NULL,
    created_at  REAL NOT NULL
);
CREATE TABLE IF NOT EXISTS machine_configs (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    name        TEXT NOT NULL,
    version     INTEGER NOT NULL,
    config_json TEXT NOT NULL,
    note        TEXT DEFAULT '',
    created_at  REAL NOT NULL,
    UNIQUE(name, version)
);
"""


class Database:
    def __init__(self, path: str):
        self.path = path
        parent = os.path.dirname(os.path.abspath(path))
        os.makedirs(parent, exist_ok=True)
        self._conn = sqlite3.connect(path, check_same_thread=False)
        self._conn.row_factory = sqlite3.Row
        self._conn.executescript(SCHEMA)
        self._conn.commit()

    def close(self) -> None:
        self._conn.close()

    # ------------------------------------------------------------------ #
    def insert_program(self, name: str, content: str, sha: str) -> int:
        cur = self._conn.execute(
            "INSERT INTO programs(name, content, sha256, created_at) VALUES (?,?,?,?)",
            (name, content, sha, time.time()),
        )
        self._conn.commit()
        return int(cur.lastrowid)

    def get_program(self, program_id: int) -> Optional[Dict[str, Any]]:
        row = self._conn.execute(
            "SELECT * FROM programs WHERE id=?", (program_id,)
        ).fetchone()
        return dict(row) if row else None

    def find_program_by_hash(self, sha: str) -> Optional[Dict[str, Any]]:
        row = self._conn.execute(
            "SELECT * FROM programs WHERE sha256=? ORDER BY id DESC LIMIT 1", (sha,)
        ).fetchone()
        return dict(row) if row else None

    def list_programs(self) -> List[Dict[str, Any]]:
        rows = self._conn.execute(
            "SELECT id, name, sha256, created_at FROM programs ORDER BY id DESC"
        ).fetchall()
        return [dict(r) for r in rows]

    # ------------------------------------------------------------------ #
    def save_config(self, name: str, config: Dict[str, Any],
                    note: str = "") -> Tuple[int, int]:
        """保存新版本。返回 (id, version)。校验失败抛 ValueError。"""
        cfg, errors = validate_config({**config, "name": name})
        if errors:
            raise ValueError("；".join(errors))
        row = self._conn.execute(
            "SELECT MAX(version) AS v FROM machine_configs WHERE name=?", (name,)
        ).fetchone()
        version = (row["v"] or 0) + 1
        cfg["version"] = version
        cur = self._conn.execute(
            "INSERT INTO machine_configs(name, version, config_json, note, created_at)"
            " VALUES (?,?,?,?,?)",
            (name, version, json.dumps(cfg, ensure_ascii=False), note, time.time()),
        )
        self._conn.commit()
        return int(cur.lastrowid), version

    def get_config(self, config_id: int) -> Optional[Dict[str, Any]]:
        row = self._conn.execute(
            "SELECT * FROM machine_configs WHERE id=?", (config_id,)
        ).fetchone()
        if not row:
            return None
        d = dict(row)
        d["config"] = json.loads(d.pop("config_json"))
        return d

    def get_config_by_version(self, name: str, version: int) -> Optional[Dict[str, Any]]:
        row = self._conn.execute(
            "SELECT * FROM machine_configs WHERE name=? AND version=?", (name, version)
        ).fetchone()
        if not row:
            return None
        d = dict(row)
        d["config"] = json.loads(d.pop("config_json"))
        return d

    def latest_version(self, name: str) -> Optional[int]:
        row = self._conn.execute(
            "SELECT MAX(version) AS v FROM machine_configs WHERE name=?", (name,)
        ).fetchone()
        return row["v"] if row and row["v"] is not None else None

    def resolve_config_ref(self, name: str,
                          version: Optional[int]) -> Optional[Dict[str, Any]]:
        """按 name + 可选版本取配置；version 为空取最新。"""
        v = version if version is not None else self.latest_version(name)
        if v is None:
            return None
        return self.get_config_by_version(name, v)

    def list_configs(self) -> List[Dict[str, Any]]:
        rows = self._conn.execute(
            "SELECT id, name, version, note, created_at FROM machine_configs"
            " ORDER BY name, version"
        ).fetchall()
        return [dict(r) for r in rows]
