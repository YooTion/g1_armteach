import struct
import io

FILE_HEADER_FORMAT = "<3s d"
FRAME_FORMAT = "<7f 7f"

class ArmDataLogger:
    def __init__(self, filename, sample_period):
        self.filename = filename
        self.sample_period = sample_period
        self.file = open(filename, "wb")
        self.buf = io.BufferedWriter(self.file, buffer_size=8192)
        self.buf.write(struct.pack(FILE_HEADER_FORMAT, b'ARM', sample_period))

    def log_frame(self, left_angles, right_angles):
        if len(left_angles) != 7 or len(right_angles) != 7:
            raise ValueError("每个机械臂必须有7个关节角度")
        self.buf.write(struct.pack(FRAME_FORMAT, *left_angles, *right_angles))

    def close(self):
        self.buf.flush()
        self.buf.close()
        self.file.close()
