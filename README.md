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
| POST | `/api/tool-life/records` | 批量导入刀具磨损记录（`tools` 数组或单条 `tool_id`+`records`） |
| GET | `/api/tool-life/records`, `/api/tool-life/records/<tool_id>` | 刀具记录汇总 / 详情 |
| POST | `/api/tool-life/predict` | 即时寿命预测（内联 `records` 或已存 `tool_id` + 本次 `condition`） |
| POST | `/api/tool-life/predict/<tool_id>` | 对已存刀具预测；加 `?download=1` 以附件导出结果 JSON |

配置支持：机床六向行程 `envelope`、G54–G59 工件零点 `work_offsets`、安全平面
`safety_clearance`、刀具表 `tools`（刀长/直径）、`h_offsets`/`d_offsets`、
快移与默认进给速度、可选 `start_position`、`arc_tolerance` 等，全部以毫米为单位。

配置保存（`POST /api/configs`）与内联审查（`POST /api/review`、
`/api/review/program/<id>`、`/api/compare`）都会校验配置：`envelope`、
`work_offsets`、`tools`、`h_offsets`、`d_offsets` 必须是对象；尺寸、坐标与
安全参数必须是有限数值（拒绝 NaN/无穷）。校验失败返回
`400 {"error": "配置校验失败：…", "details": ["…", "…"]}`，
`details` 为逐条结构化错误列表。

## 刀具寿命预测

加工任务完成后，可根据刀具历史磨损记录与本次材料、转速、进给、切削深度，
估算刀具还能安全工作多久。模块对磨损记录（累计切削时间 → 磨损量）做线性
最小二乘拟合，外推到磨钝标准（默认 VB=0.3mm）得总寿命，再用扩展 Taylor
经验公式按本次工况相对历史工况的偏离修正剩余寿命（转速/进给/切削深度指数
默认 4/2/1，可在 `life_config` 中调整）。

磨损记录字段：`cutting_time_min`（累计切削时间）、`wear_mm`（磨损量）为必传，
`speed_rpm`/`feed_mm_min`/`depth_mm`（该段历史工况）可选。批量导入示例：

```json
POST /api/tool-life/records
{"tools": [{"tool_id": "T1", "material": "45钢",
            "records": [{"cutting_time_min": 10, "wear_mm": 0.04,
                         "speed_rpm": 2000, "feed_mm_min": 300, "depth_mm": 2},
                        {"cutting_time_min": 20, "wear_mm": 0.06, "...": "..."}]}]}
```

预测请求：`condition` 为本次工况（`speed_rpm`/`feed_mm_min`/`depth_mm`，可选
`material` 与 `material_factor` 材料系数）；`life_config` 可覆盖磨钝标准、
置信水平、Taylor 指数、安全阈值（`max_speed_rpm`/`max_feed_mm_min`/
`max_depth_mm`）等。响应包含：

* `prediction`：剩余寿命 `remaining_min`、总寿命 `total_life_min` 及
  置信区间（`remaining_ci_min`/`total_life_ci_min`，delta 法由回归残差传播）；
* `factors`：转速/进给/切削深度各自的寿命影响系数、影响占比与文字说明，
  占比最高者即主要影响因素；
* `threshold_violations`：本次工况中超出安全阈值的参数及处置建议；
* `anomalies`：可解释异常提示（磨损回退疑似测量误差、记录数不足、
  R² 过低、材料不一致未修正、剩余寿命低于安全余量等）；
* `verdict`：一句话结论。

数据校验失败返回 `400 {"error": "刀具寿命数据校验失败：…", "details": [...]}`，
风格与配置校验一致。预测响应加 `?download=1` 即可以附件形式导出 JSON 结果。

## 测试

```bash
pip install -e ".[dev]"
pytest
```
