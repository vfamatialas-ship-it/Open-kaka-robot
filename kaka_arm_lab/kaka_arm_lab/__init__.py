"""
kaka_arm_lab — minimal SDK for DaMiao-motor arms over the DaMiao USB-to-CAN
adapter: MIT control + Pinocchio gravity compensation, fully config-driven.
"""
from .robot import Robot
from .motor import DmMotor, MOTOR_LIMITS
from .gravity import GravityModel
from .damiao_can import DamiaoCAN

__all__ = ["Robot", "DmMotor", "GravityModel", "DamiaoCAN", "MOTOR_LIMITS"]
__version__ = "0.1.0"
