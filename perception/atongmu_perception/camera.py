"""摄像头打开、体检，以及「当前 OpenCV 能不能弹窗」的判断。

Windows 上打开摄像头有三条路径，行为差别很大：

* ``default`` = ``CAP_ANY``（Windows 上实际解析到 Media Foundation）
* ``dshow``   = DirectShow，老牌，对内置 USB 摄像头兼容性最好
* ``msmf``    = Media Foundation，新但不稳定，部分内置摄像头会「能打开但读不出帧」

所以这里统一提供 default → dshow → msmf 的兜底，也允许显式指定后端。
"""
from __future__ import annotations

from typing import List, Optional, Tuple

import cv2
import numpy as np

CAMERA_BACKENDS = [
    ("default", cv2.CAP_ANY),
    ("dshow", getattr(cv2, "CAP_DSHOW", cv2.CAP_ANY)),
    ("msmf", getattr(cv2, "CAP_MSMF", cv2.CAP_ANY)),
]


def try_open_camera(index: int, backend_id: int, attempts: int = 10):
    """用指定后端打开摄像头并试读一帧。

    只看 ``isOpened()`` 不够——MSMF 下常常是「打开了但读不出帧」。
    成功返回 ``(cap, frame.shape)``，失败返回 ``None``。
    """
    cap = cv2.VideoCapture(index) if backend_id == cv2.CAP_ANY else cv2.VideoCapture(index, backend_id)
    if not cap.isOpened():
        cap.release()
        return None
    for _ in range(attempts):  # 有些摄像头前几帧是空的，多试几次
        ok, frame = cap.read()
        if ok and frame is not None and frame.size:
            return cap, frame.shape
    cap.release()
    return None


def open_camera(index: int, backend: str = "auto") -> Tuple[Optional[cv2.VideoCapture], Optional[str]]:
    """打开摄像头。

    ``backend='auto'`` 时依次尝试 default → dshow → msmf，第一个能真正读出一帧的才采用。
    返回 ``(cap, 实际生效的后端名)``；全部失败返回 ``(None, None)``。
    """
    for name, backend_id in CAMERA_BACKENDS:
        if backend != "auto" and name != backend:
            continue
        got = try_open_camera(index, backend_id)
        if got is not None:
            return got[0], name
        if backend != "auto":
            break
    return None, None


def probe_cameras(max_index: int = 4) -> List[Tuple[int, str]]:
    """逐个编号 × 逐个后端试，打印结果并返回 ``[(编号, 后端名), ...]``。"""
    usable = []
    for idx in range(max_index + 1):
        hits = []
        for name, backend_id in CAMERA_BACKENDS:
            got = try_open_camera(idx, backend_id)
            if got is not None:
                hits.append((name, got[1]))
                got[0].release()
        if hits:
            name, shape = hits[0]
            print("[probe] 摄像头 %d: 可用  后端=%s  分辨率 %dx%d" % (idx, name, shape[1], shape[0]))
            if len(hits) > 1:
                print("        其他可用后端: %s" % ", ".join(n for n, _ in hits[1:]))
            usable.append((idx, name))
        else:
            print("[probe] 摄像头 %d: 打不开（default / dshow / msmf 三种后端都试过）" % idx)
    return usable


def gui_available() -> bool:
    """当前 OpenCV 能不能弹窗。

    装了 ``opencv-python-headless`` 时 ``GUI: NONE``，``imshow`` 会抛
    ``cv2.error: The function is not implemented``。调用方应先问一句，
    否则会在第一帧之后直接崩掉。
    """
    try:
        cv2.imshow("__gui_probe__", np.zeros((8, 8, 3), dtype=np.uint8))
        cv2.waitKey(1)
        cv2.destroyAllWindows()
        return True
    except cv2.error:
        return False
