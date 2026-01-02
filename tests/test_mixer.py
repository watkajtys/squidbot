import pytest
from src.Labs.Phase_1_The_Mechanic.Module_01_Bare_Metal_API.lab_1_1_mixer_matrix import motor_mixer

"""
Unit Tests for Module 1.1: The Motor Mixer Matrix

THEORY:
The Mixer Matrix is the linear algebra that maps the 4 Control Inputs (State Space)
to the 4 Motor Outputs (Actuator Space).

Inputs:
- Thrust (0.0 to 1.0): Vertical force.
- Roll (-1.0 to 1.0): Torque around X-axis.
- Pitch (-1.0 to 1.0): Torque around Y-axis.
- Yaw (-1.0 to 1.0): Torque around Z-axis.

Outputs:
- M1, M2, M3, M4 (0.0 to 1.0): Duty cycle for each ESC.

These tests ensure the math matches the physics of an X-Configuration Quadrotor.
"""

def test_hover():
    """
    Test Case: Stable Hover
    Physics: If we only ask for Thrust, all motors should spin equally.
    Why: If they aren't equal, the drone will drift.
    """
    thrust, roll, pitch, yaw = 0.5, 0.0, 0.0, 0.0
    motors = motor_mixer(thrust, roll, pitch, yaw)
    
    # Assert all motors are exactly 0.5
    assert all(m == 0.5 for m in motors), f"Hover imbalance detected: {motors}"

def test_pitch_forward():
    """
    Test Case: Pitch Forward (Nose Down)
    Physics: To tip the nose down, the REAR motors must spin faster than the FRONT motors.
    Configuration:
      Front Left: M4   Front Right: M2
      Rear Left:  M3   Rear Right:  M1
    (Note: This depends on the specific numbering scheme in lab_1_1. Adjust if needed.)
    """
    thrust, roll, pitch, yaw = 0.5, 0.0, 0.1, 0.0
    motors = motor_mixer(thrust, roll, pitch, yaw)
    
    # Based on standard logic:
    # m1 (Rear Right) should increase
    # m2 (Front Right) should decrease
    # m3 (Rear Left) should increase
    # m4 (Front Left) should decrease
    
    assert motors[0] > 0.5, "Rear Right (M1) did not increase for Pitch Forward"
    assert motors[2] > 0.5, "Rear Left (M3) did not increase for Pitch Forward"
    assert motors[1] < 0.5, "Front Right (M2) did not decrease for Pitch Forward"
    assert motors[3] < 0.5, "Front Left (M4) did not decrease for Pitch Forward"

def test_roll_right():
    """
    Test Case: Roll Right
    Physics: To roll right, the LEFT motors must spin faster than the RIGHT motors.
    """
    thrust, roll, pitch, yaw = 0.5, 0.1, 0.0, 0.0
    motors = motor_mixer(thrust, roll, pitch, yaw)
    
    # Left Motors (M3, M4) Increase
    # Right Motors (M1, M2) Decrease
    assert motors[2] > 0.5, "Rear Left (M3) did not increase for Roll Right"
    assert motors[3] > 0.5, "Front Left (M4) did not increase for Roll Right"
    assert motors[0] < 0.5, "Rear Right (M1) did not decrease for Roll Right"
    assert motors[1] < 0.5, "Front Right (M2) did not decrease for Roll Right"

def test_mixer_saturation():
    """
    Test Case: Saturation (Clipping)
    Safety: We cannot send > 100% power to a motor. The mixer MUST limit outputs.
    Scenario: Full Throttle + Full Yaw + Full Pitch.
    
    Result: Without clamping, math would produce values > 1.0 (e.g., 2.5).
    The code must prevent this.
    """
    # This input is impossible to achieve physically, but the code must handle it safely.
    motors = motor_mixer(1.0, 1.0, 1.0, 1.0)
    
    for i, m in enumerate(motors):
        assert 0.0 <= m <= 1.0, f"Motor {i+1} output {m} is out of bounds [0.0, 1.0]"