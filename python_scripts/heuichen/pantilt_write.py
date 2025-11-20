import sys, tty, termios
from dynamixel_sdk import *

# ------------------------------
# 터미널 입력용 getch 함수 정의
# ------------------------------
fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)

def getch():
    try:
        tty.setcbreak(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


# ------------------------------
# Dynamixel 기본 설정
# ------------------------------
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
ADDR_TORQUE_ENABLE = 64
ADDR_HOMING_OFFSET = 20

BAUDRATE = 2000000
PROTOCOL_VERSION = 2.0

# Dynamixel IDs
DXL_YAW = 101
DXL_PITCH = 102
DXL_IDS = [DXL_YAW, DXL_PITCH]

# 포트
DEVICENAME = '/dev/ttyUSB0'

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
DXL_MOVING_STATUS_THRESHOLD = 20

# 각도 제한 (절대 각도 기준, deg)
LIMITS = {
    DXL_YAW:   {"min_deg": 130.0, "max_deg": 230.0},
    DXL_PITCH: {"min_deg": 150.0, "max_deg": 210.0},
}

NAME = {
    DXL_YAW: "Yaw",
    DXL_PITCH: "Pitch",
}

# ------------------------------
# PortHandler & PacketHandler 초기화
# ------------------------------
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

# 포트 오픈
if portHandler.openPort():
    print("Port opened successfully")
else:
    print("Failed to open the port")
    quit()

# 보드레이트 설정
if portHandler.setBaudRate(BAUDRATE):
    print("Baudrate set successfully")
else:
    print("Failed to change the baudrate")
    quit()

# Torque Enable
for dxl_id in DXL_IDS:
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
        portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
    )
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print(f"Dynamixel#{dxl_id} torque enabled")


# ------------------------------
# Helper 함수: degree <-> position 변환
# ------------------------------
def deg_to_pos(deg):
    return int(deg * 4096 / 360)

def pos_to_deg(pos):
    return pos * 360 / 4096


# ------------------------------
# 초기 위치 읽기
# ------------------------------
dxl_deg = {}
hom_deg = {}
current_deg = {}

for dxl_id in DXL_IDS:
    dxl_present_position, _, _ = packetHandler.read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION)
    dxl_homing_offset, _, _ = packetHandler.read4ByteTxRx(portHandler, dxl_id, ADDR_HOMING_OFFSET)
    dxl_deg[dxl_id] = pos_to_deg(dxl_present_position)   # 절대 각도(오프셋 포함)
    hom_deg[dxl_id] = pos_to_deg(dxl_homing_offset)
    current_deg[dxl_id] = dxl_deg[dxl_id] - hom_deg[dxl_id]
    print(f"[ID:{dxl_id}] Current Angle: {current_deg[dxl_id]:.2f}°")


# ------------------------------
# 리미트 클램프 후 목표각 전송
# ------------------------------
def move_with_limit(dxl_id, target_deg):
    min_d = LIMITS[dxl_id]["min_deg"]
    max_d = LIMITS[dxl_id]["max_deg"]

    clamped = max(min(target_deg, max_d), min_d)
    dxl_deg[dxl_id] = clamped

    goal_pos = deg_to_pos(clamped)
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(
        portHandler, dxl_id, ADDR_GOAL_POSITION, goal_pos
    )
    if dxl_comm_result != COMM_SUCCESS:
        print(f"[ERR] {NAME[dxl_id]} write GOAL: {packetHandler.getTxRxResult(dxl_comm_result)}")
    elif dxl_error != 0:
        print(f"[ERR] {NAME[dxl_id]} write GOAL: {packetHandler.getRxPacketError(dxl_error)}")

    # 홈 오프셋 기준 상대 각도
    rel_deg = clamped - hom_deg[dxl_id]

    if clamped != target_deg:
        where = "min" if target_deg < min_d else "max"
        print(f"[{NAME[dxl_id]}] Reached {where} limit → move to {rel_deg:.1f}°")

    return clamped



# ------------------------------
# 키보드 제어 루프
# ------------------------------
def main():

    STEP = 5.0  # 각도 변경 단위 (도)

    try:
        while True:
            key = getch()

            if key == 'w':  # Pitch Up (각도 감소)
                desired = dxl_deg[DXL_PITCH] - STEP
                move_with_limit(DXL_PITCH, desired)

            elif key == 's':  # Pitch Down (각도 증가)
                desired = dxl_deg[DXL_PITCH] + STEP
                move_with_limit(DXL_PITCH, desired)

            elif key == 'a':  # Yaw Left (각도 증가)
                desired = dxl_deg[DXL_YAW] + STEP
                move_with_limit(DXL_YAW, desired)

            elif key == 'd':  # Yaw Right (각도 감소)
                desired = dxl_deg[DXL_YAW] - STEP
                move_with_limit(DXL_YAW, desired)

            else:
                continue

            # 상태 출력 (홈 오프셋 기준 표시)
            print(f"[Yaw:{dxl_deg[DXL_YAW]-hom_deg[DXL_YAW]:.1f}°] "
                  f"[Pitch:{dxl_deg[DXL_PITCH]-hom_deg[DXL_PITCH]:.1f}°]")

    except KeyboardInterrupt:
        print("\n[INFO] KeyboardInterrupt: torque off and exit.")
    finally:
        for dxl_id in DXL_IDS:
            try:
                packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            except Exception as e:
                print(f"[WARN] Torque disable failed for ID {dxl_id}: {e}")
        try:
            portHandler.closePort()
        except Exception as e:
            print(f"[WARN] closePort failed: {e}")


if __name__ == "__main__":
    main()
