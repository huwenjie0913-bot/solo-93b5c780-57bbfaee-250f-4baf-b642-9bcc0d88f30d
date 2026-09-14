"""SQLite 持久化：程序、版本化机床配置、刀具磨损记录。仅使用标准库 sqlite3。"""

from __future__ import annotations

import json
import os
import sqlite3
import time
from typing import Any, Dict, List, Optional, Tuple

from .config import ConfigValidationError, validate_config

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
CREATE TABLE IF NOT EXISTS tool_life_records (
    id               INTEGER PRIMARY KEY AUTOINCREMENT,
    tool_id          TEXT NOT NULL,
    material         TEXT DEFAULT '',
    cutting_time_min REAL NOT NULL,
    wear_mm          REAL NOT NULL,
    speed_rpm        REAL,
    feed_mm_min      REAL,
    depth_mm         REAL,
    created_at       REAL NOT NULL,
    UNIQUE(tool_id, cutting_time_min)
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
        """保存新版本。返回 (id, version)。校验失败抛 ConfigValidationError。"""
        cfg, errors = validate_config({**config, "name": name})
        if errors:
            raise ConfigValidationError(errors)
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

    # ------------------------------------------------------------------ #
    def upsert_tool_life_records(self, tool_id: str, material: str,
                                 records: List[Dict[str, Any]]) -> int:
        """批量写入某刀具的磨损记录（同切削时间的旧记录被覆盖）。返回写入条数。"""
        now = time.time()
        for rec in records:
            self._conn.execute(
                "INSERT INTO tool_life_records(tool_id, material, cutting_time_min,"
                " wear_mm, speed_rpm, feed_mm_min, depth_mm, created_at)"
                " VALUES (?,?,?,?,?,?,?,?)"
                " ON CONFLICT(tool_id, cutting_time_min) DO UPDATE SET"
                " material=excluded.material, wear_mm=excluded.wear_mm,"
                " speed_rpm=excluded.speed_rpm, feed_mm_min=excluded.feed_mm_min,"
                " depth_mm=excluded.depth_mm",
                (tool_id, material, rec["cutting_time_min"], rec["wear_mm"],
                 rec.get("speed_rpm"), rec.get("feed_mm_min"),
                 rec.get("depth_mm"), now),
            )
        self._conn.commit()
        return len(records)

    def get_tool_life_records(self, tool_id: str) -> Optional[Dict[str, Any]]:
        """取某刀具的全部磨损记录（按切削时间升序）；无记录返回 None。"""
        rows = self._conn.execute(
            "SELECT * FROM tool_life_records WHERE tool_id=?"
            " ORDER BY cutting_time_min", (tool_id,),
        ).fetchall()
        if not rows:
            return None
        records = [{
            "cutting_time_min": r["cutting_time_min"],
            "wear_mm": r["wear_mm"],
            "speed_rpm": r["speed_rpm"],
            "feed_mm_min": r["feed_mm_min"],
            "depth_mm": r["depth_mm"],
        } for r in rows]
        return {"tool_id": tool_id, "material": rows[-1]["material"],
                "record_count": len(records), "records": records}

    def list_tool_life_tools(self) -> List[Dict[str, Any]]:
        """刀具磨损记录汇总列表。"""
        rows = self._conn.execute(
            "SELECT tool_id, MAX(material) AS material, COUNT(*) AS record_count,"
            " MAX(wear_mm) AS max_wear_mm, MAX(created_at) AS updated_at"
            " FROM tool_life_records GROUP BY tool_id ORDER BY tool_id"
        ).fetchall()
        return [dict(r) for r in rows]
