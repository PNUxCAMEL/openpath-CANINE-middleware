import sys, tty, termios
import numpy as np
from dynamixel_sdk import *

DXL_MOVING_STATUS_THRESHOLD = 20
TORQUE_DISABLE = 0
TORQUE_ENABLE = 1
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_HOMING_OFFSET = 20
ADDR_PRESENT_POSITION = 132

class pantiltManager():
    def __init__(self, SharedMemoryManager):
        self.shm = SharedMemoryManager
        self.position_raw = np.zeros(2)
        self.position_offset_deg = np.zeros(2)
        self.current_deg = np.zeros(2)

        # Dynamixel IDs
        self.DXL_YAW = 101
        self.DXL_PITCH = 102
        self.DXL_IDX = [0, 1]
        self.DXL_IDS = [self.DXL_YAW, self.DXL_PITCH]
        
        DEVICENAME = '/dev/ttyUSB0'
        PROTOCOL_VERSION = 2.0
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        
        self.LIMITS = {
            0: {"min_deg": 130.0, "max_deg": 230.0},
            1: {"min_deg": 150.0, "max_deg": 210.0},
        }

        self.open_dxl()
        self.read_initial_position()
        self.enable_torque()
    
    def __del__(self):
        self.disable_torque()
        self.close_dxl()

    def open_dxl(self):
        BAUDRATE = 2000000
        if self.portHandler.openPort():
            print("[PANTILT MOTOR] Port opened successfully")
        else:
            print("Failed to open the port")
            quit()

        if self.portHandler.setBaudRate(BAUDRATE):
            print("[PANTILT MOTOR] Baudrate set successfully")
        else:
            print("Failed to change the baudrate")
            quit()
    
    def close_dxl(self):
        try:
            self.portHandler.closePort()
        except Exception as e:
            print(f"[PANTILT MOTOR] closePort failed: {e}")

    def enable_torque(self):
        for dxl_id in self.DXL_IDS:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
                self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
            )
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print(f"[PANTILT MOTOR] motor-{dxl_id} torque enabled")

    def disable_torque(self):
        for dxl_id in self.DXL_IDS:
            try:
                self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            except Exception as e:
                print(f"[PANTILT MOTOR] Torque disable failed for motor-{dxl_id}: {e}")
    
    def move_with_limit(self, idx, target_deg):
        min_d = self.LIMITS[idx]["min_deg"]
        max_d = self.LIMITS[idx]["max_deg"]

        clamped = max(min(target_deg, max_d), min_d)
        goal_pos = self.deg_to_pos(clamped)

        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_IDS[idx], ADDR_GOAL_POSITION, goal_pos)
        if dxl_comm_result != COMM_SUCCESS:
            print(f"[PANTILT MOTOR] {idx} write GOAL: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            print(f"[PANTILT MOTOR] {idx} write GOAL: {self.packetHandler.getRxPacketError(dxl_error)}")

        # 홈 오프셋 기준 상대 각도
        rel_deg = clamped - self.position_offset_deg[idx]

        if clamped != target_deg:
            where = "min" if target_deg < min_d else "max"
            print(f"[PANTILT MOTOR] ID: {idx} Reached {where} limit → move to {rel_deg:.1f}°")

        return clamped
    
    def set_position(self):
        self.move_with_limit(0, self.shm.pantilt_goal_position[0] + self.position_offset_deg[0])
        self.move_with_limit(1, -self.shm.pantilt_goal_position[1] + self.position_offset_deg[1])

    def read_initial_position(self): 
        for idx in self.DXL_IDX:
            dxl_present_position, _, _ = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_IDS[idx], ADDR_PRESENT_POSITION)
            dxl_homing_offset, _, _ = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_IDS[idx], ADDR_HOMING_OFFSET)
            self.position_raw[idx] = self.pos_to_deg(dxl_present_position)   # 절대 각도(오프셋 포함)
            self.position_offset_deg[idx] = self.pos_to_deg(dxl_homing_offset)
            self.current_deg[idx] = self.position_raw[idx] - self.position_offset_deg[idx]
            # print(f"[IDX:{idx}, ID:{self.DXL_IDS[idx]}] Initial Angle: {self.current_deg[idx]:.2f}°")

    def read_position(self):
        for idx in self.DXL_IDX:
            dxl_present_position, _, _ = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_IDS[idx], ADDR_PRESENT_POSITION)
            self.position_raw[idx] = self.pos_to_deg(dxl_present_position)   # 절대 각도(오프셋 포함)
            self.current_deg[idx] = self.position_raw[idx] - self.position_offset_deg[idx]
            # print(f"[ID:{idx}] Current Angle: {self.current_deg[idx]:.2f}°")
        self.shm.pantilt_position[0] = self.current_deg[0]
        self.shm.pantilt_position[1] = -self.current_deg[1]

    def deg_to_pos(self, deg):
        return int(deg * 4096 / 360)

    def pos_to_deg(self, pos):
        return pos * 360 / 4096