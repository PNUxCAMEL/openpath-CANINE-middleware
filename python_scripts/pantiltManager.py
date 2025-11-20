import sys, tty, termios
import numpy as np
from dynamixel_sdk import *

DXL_MOVING_STATUS_THRESHOLD = 20

class pantiltManager():
    def __init__(self, SharedMemoryManager):
        self.shm = SharedMemoryManager
        
        # ------------------------------
        # Dynamixel 기본 설정
        # ------------------------------

        self.position_raw = np.zeros(2)
        self.position_offset_deg = np.zeros(2)
        self.current_deg = np.zeros(2)

        # Dynamixel IDs
        self.DXL_YAW = 101
        self.DXL_PITCH = 102
        self.DXL_IDS = [self.DXL_YAW, self.DXL_PITCH]
        
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        DEVICENAME = '/dev/ttyUSB0'
        PROTOCOL_VERSION = 2.0
        
        # 각도 제한 (절대 각도 기준, deg)
        self.LIMITS = {
            self.DXL_YAW:   {"min_deg": 130.0, "max_deg": 230.0},
            self.DXL_PITCH: {"min_deg": 150.0, "max_deg": 210.0},
        }

        self.NAME = {
            self.DXL_YAW: "Yaw",
            self.DXL_PITCH: "Pitch",
        }

        self.open_dxl()
        self.read_initial_position()
    
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
        TORQUE_ENABLE = 1
        ADDR_TORQUE_ENABLE = 64
        for dxl_id in self.DXL_IDS:
            try:
                self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
            except Exception as e:
                print(f"[PANTILT MOTOR] Torque enable failed for ID {dxl_id}: {e}")

    def disable_torque(self):
        TORQUE_ENABLE = 0
        ADDR_TORQUE_ENABLE = 64
        for dxl_id in self.DXL_IDS:
            try:
                self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
            except Exception as e:
                print(f"[PANTILT MOTOR] Torque disable failed for ID {dxl_id}: {e}")
    
    def move_with_limit(self, dxl_id, target_deg):
        ADDR_GOAL_POSITION = 116
        min_d = self.LIMITS[dxl_id]["min_deg"]
        max_d = self.LIMITS[dxl_id]["max_deg"]

        clamped = max(min(target_deg, max_d), min_d)
        goal_pos = self.deg_to_pos(clamped)
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, ADDR_GOAL_POSITION, goal_pos
        )
        if dxl_comm_result != COMM_SUCCESS:
            print(f"[ERR] {self.NAME[dxl_id]} write GOAL: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            print(f"[ERR] {self.NAME[dxl_id]} write GOAL: {self.packetHandler.getRxPacketError(dxl_error)}")

        # 홈 오프셋 기준 상대 각도
        rel_deg = clamped - self.position_offset_deg[dxl_id]

        if clamped != target_deg:
            where = "min" if target_deg < min_d else "max"
            print(f"[PANTILT MOTOR] ID: {dxl_id} Reached {where} limit → move to {rel_deg:.1f}°")

        return clamped
    
    def read_initial_position(self):
        ADDR_HOMING_OFFSET = 20
        ADDR_PRESENT_POSITION = 132
        for dxl_id in self.DXL_IDS:
            dxl_present_position, _, _ = self.packetHandler.read4ByteTxRx(self.portHandler, dxl_id, ADDR_PRESENT_POSITION)
            dxl_homing_offset, _, _ = self.packetHandler.read4ByteTxRx(self.portHandler, dxl_id, ADDR_HOMING_OFFSET)
            self.position_raw[dxl_id] = self.pos_to_deg(dxl_present_position)   # 절대 각도(오프셋 포함)
            self.position_offset_deg[dxl_id] = self.pos_to_deg(dxl_homing_offset)
            self.current_deg[dxl_id] = self.position_raw[dxl_id] - self.position_offset_deg[dxl_id]
            print(f"[ID:{dxl_id}] Current Angle: {self.current_deg[dxl_id]:.2f}°")

    def deg_to_pos(deg):
        return int(deg * 4096 / 360)

    def pos_to_deg(pos):
        return pos * 360 / 4096