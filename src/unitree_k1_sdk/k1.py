import io
import time
import struct
import math

from tqdm import tqdm
import serial
import numpy as np


DEFAULT_BAUDRATE = 1_000_000
DEFAULT_TIMEOUT = 1.0           # seconds


# Message start and end identifier specified in the UART protocol
MSG_START = b"\x55\xAA"
MSG_END = b"\x7D"


# these two value defines the upper and lower bounds of the gripper servo
# these are the raw values that the gripper servo accepts
# note that since the gripper is converting rotation to linear motion, the
# relation is not quite linear
GRIPPER_CLOSE_VALUE: int = 1250
GRIPPER_OPEN_VALUE: int = 0

JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_roll", "wrist_flex", "gripper_roll", "gripper_open"]

ZERO_POSE = np.array([
    0., 0., 0., 0., 0., 0., 0.
])

IDLE_POSE = np.array([
    0., 0.49 * np.pi, -0.5 * np.pi, 0.0, -0.1 * np.pi, 0.0, 0
])

class Message:
    TYPE_REQUEST_MESSAGE        = 0x01

class Command:
    CMD_CONTROL_END_ACTION      = 0x03
    CMD_CONTROL_ID_DEGREE       = 0x04
    CMD_CONTROL_ALL_DEGREE      = 0x05
    CMD_REPORT_SERVO_STATE      = 0x06      # 舵机状态
    CMD_CONTROL_LOCK_OR_FREE    = 0x08
    CMD_GET_VERSION             = 0x09      # 获取版本
    CMD_UPGRADE                 = 0x0A      # 升级相关的命令
    CMD_CONTROL_ALL_DEGREE_CB   = 0x0B 		# 插补控制命令
    CMD_CONTROL_ALL_DEGREE_AND_DIFF_TIME = 0x0C
    CMD_SET_SERVO_VEL           = 0x0D		# 设置舵机的插补速度
    CMD_SET_SERVO_ACC           = 0x0E		# 设置舵机的加速度
    CMD_SET_SERVO_TORQUE        = 0x0F		# 设置舵机的扭矩厌大小
    CMD_GET_SERVO_RT_INFO       = 0x10      # 获取舵机实时信息


class UnitreeK1Robot:
    """
    A Python interface for the Unitree K1 robotic arm.

    Attributes:
        port (str): The serial port to connect to the robot.
        baudrate (int): The baudrate to use for the serial connection.
    """

    @staticmethod
    def _rad_to_degree_int(rad: float) -> int:
        return int(1800. / math.pi * rad)

    @staticmethod
    def _calculate_checksum(n_bytes: int, message: bytes) -> bytes:
        sum = 0
        for i in range(n_bytes):
            sum += message[i]
        sum = sum & 0xFF
        return struct.pack("<B", sum)
    
    def __init__(self, port: str, baudrate: int = DEFAULT_BAUDRATE):
        self.port = port
        self.baudrate = baudrate

        self.port_handler = None
        self.is_connected = False
        self.is_enabled = False
        self.logs = {}

        self.joint_position_measured = np.zeros(7)
        self.joint_position_target = np.zeros(7)
    
    def connect(self):
        """
        Connect to the robot
        """
        if self.is_connected:
            raise RuntimeError("This robot device is already connected. Do not call `robot.connect()` twice.")
        
        self.port_handler = serial.Serial(self.port, self.baudrate, timeout=DEFAULT_TIMEOUT)
        # self.packet_handler
        self.is_connected = True
    
    def disconnect(self):
        """
        Disconnect from the robot
        """
        if not self.is_connected:
            raise RuntimeError("This robot device is not connected. Do not call `robot.disconnect()` twice.")
        self.port_handler.close()
        self.is_connected = False

    def reconnect(self):
        """
        Reconnect to the robot with the latest settings
        """
        if self.is_connected:
            self.disconnect()
        self.connect()

    def parse_response(self):
        msg_start = self.port_handler.read(2)
        n_bytes = self.port_handler.read(1)[0]
        message = self.port_handler.read(n_bytes)
        checksum = self.port_handler.read(1)
        msg_end = self.port_handler.read(1)

        checksum_expected = self._calculate_checksum(n_bytes, message)

        
        assert msg_start == MSG_START
        assert msg_end == MSG_END
        assert checksum == checksum_expected, f"checksum mismatch: {checksum} != {checksum_expected}"

        cmd_type = message[1]
        if cmd_type == Command.CMD_UPGRADE:
            print("升级的命令")
        elif cmd_type == Command.CMD_GET_VERSION:
            if message[0] == 0x02:
                print("version is %s" % message[2])
        elif cmd_type == Command.CMD_REPORT_SERVO_STATE:
            if message[0] == 0x01:
                joint_0_raw = struct.unpack("<h", message[2:4])[0]
                joint_1_raw = struct.unpack("<h", message[4:6])[0]
                joint_2_raw = struct.unpack("<h", message[6:8])[0]
                joint_3_raw = struct.unpack("<h", message[8:10])[0]
                joint_4_raw = struct.unpack("<h", message[10:12])[0]
                joint_5_raw = struct.unpack("<h", message[12:14])[0]
                joint_6_raw = struct.unpack("<h", message[14:16])[0]

                joint_raw = np.array([
                    joint_0_raw,
                    joint_1_raw,
                    joint_2_raw,
                    joint_3_raw,
                    joint_4_raw,
                    joint_5_raw,
                    joint_6_raw
                ], dtype=np.float32)

                self.joint_position_measured[:] = joint_raw / 1800.0 * np.pi

        elif cmd_type == Command.CMD_GET_SERVO_RT_INFO:
            print(message[0])
            if message[0] == 0x02:
                print("servo is respone CMD_GET_SERVO_RT_INFO")
                # servo_state.servo_id = mFrameBuffer[5];
                # servo_state.speed = mFrameBuffer[6]|(mFrameBuffer[7]<<8);
                # servo_state.payload = mFrameBuffer[8]|(mFrameBuffer[9]<<8);
                # servo_state.voltage = mFrameBuffer[10];
                # servo_state.current = mFrameBuffer[11]|(mFrameBuffer[12]<<8);
                # servo_state.flag = 1;
                
        else:
            print("unknown command")
            print(message)

        return message

    def send_message(self, message: bytes):
        n_bytes = len(message)
        checksum = self._calculate_checksum(n_bytes, message)
        n_bytes_bytes = struct.pack("<B", n_bytes)
        buffer = MSG_START + n_bytes_bytes + message + checksum + MSG_END
        self.port_handler.write(buffer)
    
    def _set_arm_locked(self, locked: bool):
        """
        int CSDarmCommonSerial::SendArmLockOrFree(unsigned char onoff)
        """
        locked = 1 if locked else 0
        message = struct.pack("<BBB", Message.TYPE_REQUEST_MESSAGE, Command.CMD_CONTROL_LOCK_OR_FREE, locked)
        self.send_message(message)

    def enable(self):
        """
        Locks the arm by enabling the servo control.
        """
        self._set_arm_locked(True)
        self.is_enabled = True
    
    def disable(self):
        """
        Unlocks the arm by disabling the servo control.
        """
        self._set_arm_locked(False)
        self.is_enabled = False
    
    def _set_gripper_raw(self, open: bool, value: int):
        """
        int CSDarmCommonSerial::SendArmEndAction(unsigned char onoff, short value)
        """
        open = 1 if open else 0
        # clip actuator range
        value = max(GRIPPER_OPEN_VALUE, min(value, GRIPPER_CLOSE_VALUE))

        message = struct.pack("<BBBH",
                              Message.TYPE_REQUEST_MESSAGE,
                              Command.CMD_CONTROL_END_ACTION,
                              open,
                              value)
        self.send_message(message)
    
    def set_gripper(self, value: float):
        """
        Sets the gripper open state.

        Args:
            value (float): The gripper open state. 1.0 is fully open, 0.0 is fully closed.
        """
        value = GRIPPER_OPEN_VALUE + ((1 - value) * (GRIPPER_CLOSE_VALUE - GRIPPER_OPEN_VALUE))
        value = int(value)
        self._set_gripper_raw(False, value)
    
    def set_gripper_rad(self, rad: float):
        """
        Sets the gripper open angle.

        Args:
            rad (float): The gripper open angle in radians. 0 rad is fully open, 2.0944 rad (120 deg) is fully closed.
        """
        value = rad / 2.0944
        self.set_gripper(1.0 - rad)
    

    def _send_arm_all_server_time(self, difftime: int, v1: float, v2: float, v3: float, v4: float, v5: float, v6: float):
        """
        int CSDarmCommonSerial::SendArmAllServerTime(short difftime, float v1, float v2, float v3, float v4, float v5, float v6)

        Args:
            difftime (int): The time spend to move to the target position, in milliseconds.
            v1~v6 (float): The target angles of the 6 joints, in radians.
        """
        message = struct.pack("<BB",
                              Message.TYPE_REQUEST_MESSAGE,
                              Command.CMD_CONTROL_ALL_DEGREE_AND_DIFF_TIME)
        
        message += struct.pack("<h", difftime)
        message += struct.pack("<h", self._rad_to_degree_int(v1))
        message += struct.pack("<h", self._rad_to_degree_int(v2))
        message += struct.pack("<h", self._rad_to_degree_int(v3))
        message += struct.pack("<h", self._rad_to_degree_int(v4))
        message += struct.pack("<h", self._rad_to_degree_int(v5))
        message += struct.pack("<h", self._rad_to_degree_int(v6))
        
        self.send_message(message)
    

    def update(self):
        self.parse_response()


    def send_joint_position_target(self):
        """
        Writes the target joint positions to the robot
        """
        self._send_arm_all_server_time(
            0,
            self.joint_position_target[0],
            self.joint_position_target[1],
            self.joint_position_target[2],
            self.joint_position_target[3],
            self.joint_position_target[4],
            self.joint_position_target[5]
        )
        self.set_gripper_rad(self.joint_position_target[6])

    def step(self, actions: np.ndarray):
        self.parse_response()
        self.joint_position_target[:] = actions

        self.send_joint_position_target()
        

    # ====== lerobot compatibility ======
    def are_motors_configured(self) -> bool:
        """
        Check if the motors are configured correctly.

        This function provides compatibility with the `lerobot` library.

        Returns:
            bool: True if the motors are configured correctly, False otherwise
        """
        return True
    
    @property
    def motor_names(self) -> list[str]:
        return JOINT_NAMES
    
    @property
    def motor_models(self) -> list[str]:
        return ["K1 Servo"] * 7
    
    @property
    def motor_indices(self) -> list[int]:
        return list(range(7))
    
    def read(self, data_name: str, motor_names: str | list[str] | None = None) -> np.ndarray:
        if data_name == "Torque_Enable":
            return self.is_enabled
        elif data_name == "Present_Position":
            return self.joint_position_measured
        elif data_name == "Goal_Position":
            return self.joint_position_target
        else:
            raise ValueError(f"Unknown data name: {data_name}")
        
    def write(self, data_name: str, values: int | float | np.ndarray, motor_names: str | list[str] | None = None) -> None:
        if data_name == "Torque_Enable":
            self.enable()
        elif data_name == "Goal_Position":
            self.joint_position_target[:] = values
            self.send_joint_position_target()
        else:
            raise ValueError(f"Unknown data name: {data_name}")

