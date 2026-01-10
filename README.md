# IR Pen Tracker

IR-pen-tracker: 使用红外反光标记进行笔姿态与轨迹追踪，支持 Kinect 与 Intel RealSense。

功能概览
- 支持 Kinect（pyk4a）与 Intel RealSense（pyrealsense2）相机采集
- IR 笔追踪（基于两点反光标记 + 深度反投影 + 线拟合）
- 桌面平面标定与世界坐标可视化
- 录制彩色视频、深度/IR 原始图及笔轨数据

快速开始
1. 安装依赖
   - pip install -e .
   - pip install .[test]
2. 运行主程序
   - python -m ir_pen_tracker.main
3. 预览 RealSense 深度/IR
   - python tmpscripts/preview_realsense_depth.py

配置说明
- config.json 示例：
  {
    "camera": {
      "type": "realsense",
      "realsense": {
        "preset": "high_accuracy",
        "use_max_resolution": true,
        "fps": 30,
        "enable_ir": true
      }
    },
    "debug": {
      "skip_calibration": false
    }
  }

许可证
- MIT License
