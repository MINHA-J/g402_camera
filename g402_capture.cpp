import serial
import struct
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

# CDC 포트 경로
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200  # 무시될 수 있음

# 버스트 데이터 포맷(9바이트) : motion, dx, dy, squal, shutter_hi, shutter_lo, pix_max, pix_accum, pix_min
BURST_FORMAT = "<BbbBBBBBB"
BURST_SIZE = struct.calcsize(BURST_FORMAT)

# 실시간 데이터 저장 버퍼
MAX_POINTS = 200
dx_buffer = deque(maxlen = MAX_POINTS)
dy_buffer = deque(maxlen = MAX_POINTS)
squal_buffer = deque(maxlen = MAX_POINTS)
shutter_buffer = deque(maxlen = MAX_POINTS)
pix_min_buffer = deque(maxlen = MAX_POINTS)
pix_max_buffer = deque(maxlen = MAX_POINTS)
pix_accum_buffer = deque(maxlen = MAX_POINTS)

# 시리얼 포트 초기화
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout = 0.1)

# matplotlib 초기화
fig, (ax_motion, ax_info) = plt.subplots(2, 1, figsize = (10, 6))
line_squal, = ax_info.plot([], [], label = 'SQUAL')
line_shutter, = ax_info.plot([], [], label = 'Shutter')
line_min, = ax_info.plot([], [], label = 'Pix_Min')
line_max, = ax_info.plot([], [], label = 'Pix_Max')
line_accum, = ax_info.plot([], [], label = 'Pix_Accum')
ax_info.legend()
ax_info.set_ylim(0, 300)

quiver = ax_motion.quiver([], [], [], [], scale = 1, scale_units = 'xy', angles = 'xy')

motion_x = []
motion_y = []

fps_count = 0
last_fps_time = time.time()

def update(frame) :
    global motion_x, motion_y, fps_count, last_fps_time

    # 데이터 수신
    raw = ser.read(BURST_SIZE)
    if len(raw) != BURST_SIZE:
return

data = struct.unpack(BURST_FORMAT, raw)
motion, dx, dy, squal, shut_hi, shut_lo, pix_max, pix_acc, pix_min = data
shutter = (shut_hi << 8) | shut_lo

# 버퍼 업데이트
dx_buffer.append(dx)
dy_buffer.append(dy)
squal_buffer.append(squal)
shutter_buffer.append(shutter)
pix_min_buffer.append(pix_min)
pix_max_buffer.append(pix_max)
pix_accum_buffer.append(pix_acc)

# 2D 움직임 누적
last_x = motion_x[-1] if motion_x else 0
last_y = motion_y[-1] if motion_y else 0
motion_x.append(last_x + dx)
motion_y.append(last_y + dy)
if len(motion_x) > MAX_POINTS:
motion_x = motion_x[-MAX_POINTS:]
motion_y = motion_y[-MAX_POINTS:]

# FPS 계산
fps_count += 1
now = time.time()
if now - last_fps_time >= 1:
fps = fps_count / (now - last_fps_time)
print(f"[FPS] {fps:.2f}")
fps_count = 0
last_fps_time = now

# 플롯 업데이트
ax_motion.clear()
ax_motion.set_title("Motion Path (ΔX, ΔY)")
ax_motion.plot(motion_x, motion_y, color = 'blue')
ax_motion.set_xlim(min(motion_x + [0]) - 10, max(motion_x + [0]) + 10)
ax_motion.set_ylim(min(motion_y + [0]) - 10, max(motion_y + [0]) + 10)

# 센서 정보 플롯
line_squal.set_data(range(len(squal_buffer)), list(squal_buffer))
line_shutter.set_data(range(len(shutter_buffer)), list(shutter_buffer))
line_min.set_data(range(len(pix_min_buffer)), list(pix_min_buffer))
line_max.set_data(range(len(pix_max_buffer)), list(pix_max_buffer))
line_accum.set_data(range(len(pix_accum_buffer)), list(pix_accum_buffer))

ax_info.set_xlim(0, len(squal_buffer) if squal_buffer else 100)

ani = animation.FuncAnimation(fig, update, interval = 10)
plt.tight_layout()
plt.show()
