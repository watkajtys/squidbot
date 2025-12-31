"""
Professional Unit Testing
Goal: Verify the logic of your labs before they ever touch the drone.
Run this with: `pytest tests/test_pid.py`
"""

import unittest
from src.labs.lab_5_pid import PIDController

class TestPID(unittest.TestCase):
    def test_proportional_action(self):
        # Setup a PID with only P-gain
        pid = PIDController(kp=1.0, ki=0.0, kd=0.0)
        
        # If setpoint=1.0 and measurement=0.0, output should be 1.0
        output = pid.update(setpoint=1.0, measurement=0.0, current_time=0.01)
        
        # Note: This will fail until you implement the TODOs in the lab stub!
        self.assertEqual(output, 1.0)

    def test_steady_state(self):
        pid = PIDController(kp=1.0, ki=0.0, kd=0.0)
        
        # If we are already at the target, output should be 0.0
        output = pid.update(setpoint=1.0, measurement=1.0, current_time=0.01)
        self.assertEqual(output, 0.0)

if __name__ == '__main__':
    unittest.main()
 Lands.
