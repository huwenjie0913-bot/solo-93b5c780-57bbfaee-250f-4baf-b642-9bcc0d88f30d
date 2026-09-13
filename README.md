# G-code 静态审查服务

机加工程序换到新设备时，工件零点、刀长/刀径补偿与机床行程的微小差异都可能在首件
试切前埋下撞机风险。本服务在**不实际运行机床**的前提下，上传 G-code 并按指定的
机床配置重建逐段运动轨迹，静态诊断越界、低空快移、未定义刀具、圆弧几何错误与危险
模态继承，所有结论都定位到行号并给出判定依据和前置模态状态。

## 技术栈

Python 3 · Flask · SQLite（仅标准库 `sqlite3`）。

## 安装与启动

```bash
pip install -r requirements.txt        # 或 pip install .
python -m gcode_review --port 5000     # 模块入口
gcode-review --port 5000               # 安装后的控制台脚本
```

参数：`--host`、`--port`、`--db`（SQLite 路径，也可用环境变量
`GCODE_DB_PATH` / `HOST` / `PORT`）。

## 审查能力

解析 G00/G01/G02/G03、G20/G21 单位、G90/G91 绝对/增量、G17/G18/G19 平面、
G41/G42/G40 刀径补偿、G43/G44/G49 刀长补偿、G54–G59、G52、G92、固定循环
G73/G74/G76/G81–G89、G28/G30、G04 等；内部统一换算为毫米与机床物理坐标。

诊断项包括：

| 代码 | 含义 |
| --- | --- |
| `ENVELOPE_VIOLATION` | 运动（含刀具半径）超出机床行程包络 |
| `RAPID_BELOW_SAFETY` | G00 快移低于安全平面（区分 XY 联动 / 垂直下插 / 上抬退刀） |
| `UNDEFINED_TOOL` / `UNDEFINED_TOOL_LENGTH` / `UNDEFINED_CUTTER_RADIUS` | 未定义刀具、H 刀长或 D 刀径补偿 |
| `ARC_GEOMETRY_ERROR` | 起终点半径不一致、弦长大于直径、整圆用 R 等 |
| 模态继承类 | `SPINDLE_OFF_CUTTING`、`CANNED_ACTIVE_AT_END`、`CUTTER_COMP_ACTIVE_AT_END`、`G92_*`、`IMPLICIT_MOTION_MODE` 等 |

响应包含：风险等级与计数、逐项诊断（行号、判定依据 `basis`、执行前模态
`preceding_state`、关联段索引）、轨迹边界（机床坐标与工件坐标）、估算耗时
（快移/切削/暂停/换刀分项及计算口径）、重建后的全部运动段。

## 主要 API

| 方法 | 路径 | 说明 |
| --- | --- | --- |
| GET | `/health` | 健康检查 |
| POST | `/api/programs` | 上传程序（multipart `file` 或 JSON `name`+`content`） |
| GET | `/api/programs`, `/api/programs/<id>` | 列表 / 详情 |
| POST | `/api/configs` | 保存机床配置，同名自动递增版本 |
| GET | `/api/configs`, `/api/configs/<id>` | 版本列表 / 详情 |
| POST | `/api/review` | 即时审查（内联 `content` 或 `program_id` + 配置引用/内联配置） |
| POST | `/api/review/program/<id>` | 对已存程序审查 |
| POST | `/api/compare` | 同一程序在两套配置（`config_a`/`config_b`）下的风险差异 |

配置支持：机床六向行程 `envelope`、G54–G59 工件零点 `work_offsets`、安全平面
`safety_clearance`、刀具表 `tools`（刀长/直径）、`h_offsets`/`d_offsets`、
快移与默认进给速度、可选 `start_position`、`arc_tolerance` 等，全部以毫米为单位。

## 测试

```bash
pip install -e ".[dev]"
pytest
```
