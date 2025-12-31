from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Standard Launch File for the Squid Drone.
    Starts the core control and safety nodes.
    """
    return LaunchDescription([
        # The Motor Mixer: Translates Twist commands to PWM
        Node(
            package='squid_control',
            executable='motor_mixer',
            name='motor_mixer_node',
            output='screen',
            parameters=[
                {'mixer_type': 'X_FRAME'},
                {'pwm_frequency': 400}
            ]
        ),
        
        # Placeholder for the State Estimator (EKF)
        # Students will implement this in Module 7
        Node(
            package='squid_control',
            executable='state_estimator',
            name='ekf_node',
            output='screen',
            condition=None # Enabled once students implement it
        ),
    ])
