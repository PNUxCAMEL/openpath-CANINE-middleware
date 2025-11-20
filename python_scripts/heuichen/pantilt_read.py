import os
import time
import sys, tty, termios
from dynamixel_sdk import *


# ------------------------------
# 터미널 입력용 getch 함수 정의
# ------------------------------
fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)

def getch():
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


# ------------------------------
# Dynamixel 기본 설정
# ------------------------------
ADDR_HOMING_OFFSET = 20
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132

BAUDRATE = 2000000
PROTOCOL_VERSION = 2.0

# Dynamixel IDs
DXL_YAW = 101
DXL_PITCH = 102
DXL_IDS = [DXL_YAW, DXL_PITCH]

# Port
DEVICENAME = '/dev/ttyUSB0'

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
DXL_MOVING_STATUS_THRESHOLD = 20

# angle limit
LIMITS = {
    DXL_YAW:   {"min_deg": -50.0, "max_deg": 50.0},
    DXL_PITCH: {"min_deg": -30.0, "max_deg": 30.0},
}

# ------------------------------
# Port setting
# ------------------------------
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    quit()

if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    quit()

for dxl_id in DXL_IDS:
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
        portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
    )
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print(f"Dynamixel#{dxl_id} has been successfully connected")


# ------------------------------
# Helper 함수: degree <-> position 변환
# ------------------------------
def deg_to_pos(deg):
    return int(deg * 4096 / 360)

def pos_to_deg(pos):
    return pos * 360 / 4096


# ------------------------------
# 현재 위치 읽기 루프
# ------------------------------
def main():
    
    STEP = 5.0

    try:
        while True:
            positions = []
            for dxl_id in DXL_IDS:
                dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(
                    portHandler, dxl_id, ADDR_PRESENT_POSITION
                )
                dxl_homing_offset, _, _ = packetHandler.read4ByteTxRx(
                    portHandler, dxl_id, ADDR_HOMING_OFFSET
                )
                dxl_position = dxl_present_position - dxl_homing_offset
        
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % packetHandler.getRxPacketError(dxl_error))

                positions.append(dxl_position)

            angles = [pos * 360 / 4096 for pos in positions]

            print(
                "[ID:%03d] %.2f°  [ID:%03d] %.2f°"
                % (DXL_IDS[0], angles[0], DXL_IDS[1], angles[1])
            )
                
    
    except KeyboardInterrupt:
        print("\n[INFO] KeyboardInterrupt: torque off and exit.")
    finally:
        for i in DXL_IDS:
            packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)


if __name__ == "__main__":
    main()