"""sensor_msgs/Image -> OpenCV BGR without cv_bridge (avoids numpy ABI issues)."""
import cv2
import numpy as np


def image_to_bgr(msg):
    enc = msg.encoding.lower()
    buf = np.frombuffer(msg.data, np.uint8)
    h, w, step = msg.height, msg.width, msg.step
    if enc in ('rgb8', 'bgr8'):
        img = buf.reshape(h, step)[:, :w * 3].reshape(h, w, 3)
        return np.ascontiguousarray(img[:, :, ::-1] if enc == 'rgb8' else img)
    if enc in ('rgba8', 'bgra8'):
        img = buf.reshape(h, step)[:, :w * 4].reshape(h, w, 4)
        return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR if enc == 'rgba8' else cv2.COLOR_BGRA2BGR)
    if enc == 'mono8':
        return cv2.cvtColor(buf.reshape(h, step)[:, :w], cv2.COLOR_GRAY2BGR)
    if enc in ('yuv422_yuy2', 'yuyv'):
        return cv2.cvtColor(buf.reshape(h, step)[:, :w * 2].reshape(h, w, 2), cv2.COLOR_YUV2BGR_YUY2)
    return None
