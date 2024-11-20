
from unitree_k1_sdk import UnitreeK1Robot


if __name__ == "__main__":
    arm = UnitreeK1Robot("/dev/ttyACM0")
    arm.connect()

    # enable the motors of the robot
    arm.enable()

    # update the arm state from the robot
    arm.update()

    print("Present Position:", arm.joint_position_measured)

    # disable the motors of the robot
    # WARNING: the robot will collapse when the motors are disabled
    arm.disable()

