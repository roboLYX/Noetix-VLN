# progress_dir

## 本轮任务目标

- 验证 Unitree Go2-W 在 LocalAP 模式下的 LiDAR 点云回传能力。
- 验证 LiDAR 订阅与保守控制命令并发时，点云回传是否仍稳定。
- 将本轮新增脚本、验证口径、遗留问题和后续建议集中记录到本文件，便于后续对话直接接着推进。

## 本轮新增文件

- `scripts/benchmark_lidar_stream.py`
- `scripts/benchmark_mixed_load.py`
- `unitree_webrtc_connect/utils/benchmark_metrics.py`
- `progress_dir.md`

## 本轮修改文件

- 无

## 每个文件的改动说明

- `scripts/benchmark_lidar_stream.py`
  - 新增 LocalAP 模式 LiDAR 单链路 benchmark。
  - 订阅 `RTC_TOPIC["ULIDAR_ARRAY"]`，发布 `RTC_TOPIC["ULIDAR_SWITCH"]="on"`。
  - 逐帧记录本地 `time.monotonic()` 到达时间、解码是否成功、点数、错误信息。
  - 输出总帧数、平均频率、median/p95/max gap、超过阈值的 gap 次数、平均/中位点数。
  - 支持 `--duration`、`--gap-threshold`、`--csv`、`--decoder`、`--connect-timeout`、`--request-timeout`。
  - 新增 `--output`，默认把终端摘要同步写入 `benchmark_lidar_stream_output.txt`。
  - 支持 Ctrl+C 退出并在 `finally` 里关闭 LiDAR 和 WebRTC 连接。

- `scripts/benchmark_mixed_load.py`
  - 新增 LocalAP 模式 LiDAR + 控制并发 benchmark。
  - 同时订阅 `ULIDAR_ARRAY`、`LOW_STATE`、`LF_SPORT_MOD_STATE`。
  - 默认仅周期性发送 `StopMove`，不执行危险动作。
  - 可选通过 `--yaw-pulse` 开启很小的交替 yaw 脉冲，默认关闭。
  - 记录 LiDAR gap/frequency、控制请求 RTT、状态 topic 回调帧数。
  - 输出简洁终端摘要，并默认同步写入 `benchmark_mixed_load_output.txt`。
  - 在无 LiDAR 帧/全失败时明确打印“LiDAR 不可用”。

- `unitree_webrtc_connect/utils/benchmark_metrics.py`
  - 新增 LiDAR 帧级统计与控制 RTT 统计工具。
  - 提供 `LidarStreamMetrics` 和 `RequestRttMetrics`，支持摘要计算和 LiDAR CSV 导出。

- `progress_dir.md`
  - 新增本轮进展记录文件。

## 当前验证结果

- 代码实现已完成。
- 已完成脚本级语法检查、统计工具导入检查、两个 benchmark 脚本的 `--help` 参数检查。
- 本轮在当前开发容器中未直连实体 Go2-W 执行 LocalAP 实机 benchmark，因此尚未产出真实 LiDAR 稳定性结论。
- 后续需要在 Go2-W LocalAP 实机环境运行下面的 benchmark 命令获取结论。

## 当前遗留问题

- 需要在真实 Go2-W LocalAP 环境执行两个 benchmark，确认 `ULIDAR_ARRAY` 在 Go2-W 上是否可用、点数是否非空、gap 是否满足 DimOS 侧规划需求。
- 如果 `benchmark_lidar_stream.py` 显示 0 帧或全失败，需要进一步确认 Go2-W 是否开放 `rt/utlidar/voxel_map_compressed`，以及当前机型/固件是否兼容此 LiDAR topic 和解码器。
- 当前脚本只能统计 pub/sub 回调层面已经送达的 LiDAR 消息；如果底层二进制解码在 `webrtc_datachannel.py` 内部直接异常并被日志吞掉，需要再考虑给 datachannel 增加更显式的 decode error hook。

## 下一步建议

- 先在 Go2-W LocalAP 网络下跑 `benchmark_lidar_stream.py`，确认 LiDAR 是否有帧、频率和点数是否稳定。
- 再跑 `benchmark_mixed_load.py` 默认 StopMove 并发测试，观察 LiDAR gap 与控制 RTT 是否出现明显退化。
- 如果 LiDAR 单链路可用，再按 DimOS 需求补一个 RGB benchmark 和 LiDAR/RGB/状态/控制混合压力测试。
- 如果 LiDAR 单链路不可用，优先排查 Go2-W 机型/固件对 `ULIDAR_ARRAY` topic 的支持情况，而不是继续扩动作脚本。

## 测试命令

```bash
cd /home/user/unitree_webrtc_connect

# 本地静态检查
PYTHONPYCACHEPREFIX=/tmp/unitree_pycache python -m py_compile \
  scripts/benchmark_lidar_stream.py \
  scripts/benchmark_mixed_load.py \
  unitree_webrtc_connect/utils/benchmark_metrics.py

PYTHONDONTWRITEBYTECODE=1 python scripts/benchmark_lidar_stream.py --help
PYTHONDONTWRITEBYTECODE=1 python scripts/benchmark_mixed_load.py --help

# LiDAR 单链路 benchmark
python scripts/benchmark_lidar_stream.py \
  --duration 30 \
  --gap-threshold 0.5 \
  --csv lidar_benchmark_localap.csv \
  --output benchmark_lidar_stream_output.txt

# LiDAR + 保守控制并发 benchmark，默认只发 StopMove
python scripts/benchmark_mixed_load.py \
  --duration 30 \
  --gap-threshold 0.5 \
  --control-interval 1.0 \
  --output benchmark_mixed_load_output.txt

# 可选：开启很小的交替 yaw 脉冲，默认关闭，只有确认场地安全后再试
python scripts/benchmark_mixed_load.py \
  --duration 30 \
  --gap-threshold 0.5 \
  --control-interval 1.0 \
  --yaw-pulse 0.08 \
  --yaw-duration 0.15 \
  --output benchmark_mixed_load_output.txt
```

## 输出文件

- `lidar_benchmark_localap.csv`
  - 由 `scripts/benchmark_lidar_stream.py --csv lidar_benchmark_localap.csv` 生成。
  - 字段包含 `frame_index`、`arrival_monotonic_s`、`delta_s`、`decoded_ok`、`point_count`、`error`。
- `benchmark_lidar_stream_output.txt`
  - 由 `scripts/benchmark_lidar_stream.py` 默认生成，记录 LiDAR benchmark 摘要。
- `benchmark_mixed_load_output.txt`
  - 由 `scripts/benchmark_mixed_load.py` 默认生成，记录混合负载 benchmark 摘要。
