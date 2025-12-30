import struct
import time
# TODO: Import serial library

class MSPDriver:
    def __init__(self, port='/dev/ttyS0', baudrate=115200):
        """
        Initialize UART connection to Flight Controller.
        """
        self.port = port
        self.baudrate = baudrate
        # TODO: Open serial connection
        pass

    def _calculate_checksum(self, data):
        """
        XOR checksum for MSP protocol.
        """
        xor_sum = 0
        # TODO: Iterate over data bytes and XOR them
        return xor_sum

    def send_motor_command(self, motor_values):
        """
        Send MSP_SET_MOTOR command.
        motor_values: List of 4 integers [m1, m2, m3, m4] (1000-2000)
        """
        # TODO: Construct the byte packet
        # Header: $M<
        # Size: ...
        # Type: 214 (MSP_SET_MOTOR)
        # Payload: ...
        # Checksum: ...
        pass

    def arm(self):
        """
        Send auxiliary channel command to ARM the drone.
        """
        pass

    def disarm(self):
        """
        Send auxiliary channel command to DISARM.
        """
        pass
