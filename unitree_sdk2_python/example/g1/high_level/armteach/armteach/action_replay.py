import struct
import io
import time
from .action_logger import FILE_HEADER_FORMAT, FRAME_FORMAT

def replay_arm_file(filename, callback=None):
    header_size = struct.calcsize(FILE_HEADER_FORMAT)
    frame_size = struct.calcsize(FRAME_FORMAT)

    with open(filename, "rb") as f:
        buf = io.BufferedReader(f, buffer_size=65536)
        header_data = buf.read(header_size)
        start_flag, sample_period = struct.unpack(FILE_HEADER_FORMAT, header_data)
        if start_flag != b'ARM':
            raise ValueError("文件头错误")

        while True:
            frame_data = buf.read(frame_size)
            if not frame_data:
                break
            *angles, = struct.unpack(FRAME_FORMAT, frame_data)
            left = angles[:7]
            right = angles[7:]

            if callback:
                callback(left, right)
            else:
                print(f"[回放] 左手={left} 右手={right}")

            time.sleep(sample_period)


def iter_arm_frames(filename):
    """
    逐帧读取机械臂二进制示教文件
    每次 yield (left[7], right[7])
    """
    header_size = struct.calcsize(FILE_HEADER_FORMAT)
    frame_size = struct.calcsize(FRAME_FORMAT)

    with open(filename, "rb") as f:
        buf = io.BufferedReader(f, buffer_size=65536)

        # 读取文件头
        header_data = buf.read(header_size)
        start_flag, sample_period = struct.unpack(FILE_HEADER_FORMAT, header_data)
        if start_flag != b'ARM':
            raise ValueError("文件头错误")

        # 一帧一帧读取
        while True:
            frame_data = buf.read(frame_size)
            if not frame_data:
                break

            *angles, = struct.unpack(FRAME_FORMAT, frame_data)
            left = angles[:7]
            right = angles[7:]
            yield left, right, sample_period
