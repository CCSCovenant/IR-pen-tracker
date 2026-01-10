import sys
import time
from typing import Optional

def _exit(code: int, msg: Optional[str] = None):
    if msg:
        print(msg)
    sys.exit(code)

def main():
    try:
        from pyk4a import PyK4A, Config, ColorResolution, ImageFormat, FPS, DepthMode, connected_device_count
    except Exception as e:
        _exit(1, f"依赖未就绪或导入失败: {e}")

    try:
        cnt = connected_device_count()
    except Exception as e:
        _exit(2, f"设备查询失败: {e}")

    if cnt == 0:
        _exit(2, "未检测到 Azure Kinect 设备")

    cfg = Config(
        color_resolution=ColorResolution.RES_720P,
        color_format=ImageFormat.COLOR_NV12,
        camera_fps=FPS.FPS_5,
        depth_mode=DepthMode.NFOV_UNBINNED,
        synchronized_images_only=False,
    )

    cam = PyK4A(cfg)

    try:
        cam.start()
    except Exception as e:
        _exit(3, f"设备启动失败: {e}")

    n = 10
    ok = 0
    t0 = None
    t_last = None
    for i in range(n):
        cap = cam.get_capture()
        if cap.color is not None:
            ok += 1
        t = time.time()
        if t0 is None:
            t0 = t
        t_last = t

    try:
        cam.stop()
    except Exception:
        pass

    elapsed = (t_last - t0) if (t_last and t0) else 0.0
    fps = (n - 1) / elapsed if elapsed > 0 else 0.0

    print("设备数量:", cnt)
    print("当前配置: 720p / 5fps / NV12 / NFOV_UNBINNED")
    print("采集成功帧数:", ok, "/", n)
    print("估算FPS:", round(fps, 2))

    if ok != n:
        _exit(4, "状态: 异常 (存在空帧)")
    if fps < 3.0 or fps > 7.0:
        _exit(4, "状态: 异常 (FPS不在预期范围)")

    _exit(0, "状态: 正常")

if __name__ == "__main__":
    main()
