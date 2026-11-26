# 云桌面观看与MP4录制

部署目录 `/home/admin/atongmu_deploy_20261120`，以下命令在这台服务器的SSH终端执行。
窗口显示在云电脑现有的 admin 图形桌面，不在SSH终端、本地浏览器或本地Isaac Sim中显示。

## 观看小车自动导航

先打开云电脑桌面，保持桌面会话已登录，然后执行：

```bash
bash /home/admin/atongmu_deploy_20261120/demo_tools/launch_demo.sh watch_NEW_ID gui
```

将 `watch_NEW_ID` 换成一个未使用过的英文、数字或下划线ID。脚本会打开
`Isaac Sim Python 4.5.0` 窗口，使用独立跟随观察相机，启动视觉和导航，等初始化完成后
依次导航到 `(0,7,π/2)` 和 `(-3,7,π)`。任务完成后停车，窗口保持打开。

启动可能需数分钟。程序负责播放和控制，无须再点Play、切换场景或启动第二个仿真。
只打开窗口但不自动导航可使用已有 `start_demo.sh NEW_ID gui`。

## 自动录制MP4

结束上一组演示后，用新ID执行：

```bash
bash /home/admin/atongmu_deploy_20261120/demo_tools/launch_demo.sh record_NEW_ID record
```

录制模式也会打开云桌面窗口、执行上述两个导航目标。结束后自动停车、关闭本次仿真并合成：

```text
/home/admin/jetbot-warehouse/artifacts/record_NEW_ID/observer_video/navigation_1x.mp4
```

脚本由独立tmux会话管理，SSH断线不会停止已启动的演示流程。录的是观察相机画面，
不包含整个云桌面或SSH终端。使用原始仿真采集时间恢复1倍速，独立于云端墙钟运行速度。
原图帧、frames.csv、录制SUMMARY、ENCODING_REPORT和VISIBLE_DEMO_RESULT均保留。
若任务失败也会保存已有画面，需根据VISIBLE_DEMO_RESULT判断，不能仅凭MP4存在称导航成功。

查看运行日志：

```bash
tail -f /home/admin/atongmu_deploy_20261120/logs/record_NEW_ID.demo.log
```

也可以在云桌面文件管理器进入上述目录双击MP4。在自己的电脑下载时，从**本地终端**执行：

```bash
scp -P 58046 root@SERVER_HOST:/home/admin/jetbot-warehouse/artifacts/record_NEW_ID/observer_video/navigation_1x.mp4 .
```

## 停止、手动合成和加速版

一次只运行一组仿真。观看完GUI演示、或需要提前结束时执行：

```bash
bash /home/admin/atongmu_deploy_20261120/stop_demo.sh
```

手动停止时可等待case目录出现RUNNER_RESULT.json，再按需合成已有帧：

```bash
python3 /home/admin/atongmu_deploy_20261120/demo_tools/encode_observer_video.py \
  --case /home/admin/jetbot-warehouse/artifacts/record_NEW_ID --speed 1
```

如需4倍速，用 `--speed 4` 生成 `navigation_4x.mp4`。脚本拒绝覆盖已有同名视频。
原录像模块CONTRACT中保留的4倍速是旧版逐帧25fps编码约定；新编码器按frames.csv实际
仿真间隔和指定speed计算播放时长，以ENCODING_REPORT为准。

失败排查优先看runner.log、VISIBLE_DEMO_RESULT.json及保护门fault，不修改保护阈值强行继续。
录像与深度同时启用需要实测负载，本次导航结果保留在对应case中；不是长时间稳定性承诺。

## 本次实际验证

case为 `gui_record_20261120_01`，云桌面已创建Isaac窗口并完成两次原生NavigateToPose，
均SUCCEEDED，保护门无fault、输入审计通过。结束后停车零速并正常关闭仿真。
录像采集337帧，按实际仿真间隔合成为1280×720、25fps、H.264/yuv420p MP4，
53.96秒、8,867,944字节。完整ffmpeg解码无错误，本地/云端文件SHA256一致，
抽查5秒、28秒及46秒画面确认小车和跟随视角正常。视频与小型证据已取回
`experiments/server_deploy_20261120/video_demo_01/`。

首次有窗口启动在资源解析处等待较久，随后进入导航；已保留启动日志。
当前代码不会通过修改控制保护阈值来掩盖启动或性能问题。
