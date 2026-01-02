"""
Lab 1.5: The Virtual Dyno
Goal: Visualizing the Mixer Matrix before risking hardware.

Description:
This script acts as a "Simulator" for your mixer logic. 
It captures keyboard inputs (WASD + Arrows) to simulate a pilot's stick commands.
It feeds these commands into YOUR `motor_mixer` function.
It renders the output as high-fidelity progress bars with numerical precision.

Prerequisites:
    pip install rich readchar
"""

import sys
import os
import time
from rich.live import Live
from rich.table import Table
from rich.progress import Progress, BarColumn, TextColumn
from rich.layout import Layout
from rich.panel import Panel
from rich.console import Console

# Import your mixer logic
try:
    from lab_1_1_mixer_matrix import motor_mixer
except ImportError:
    print("Error: Could not import lab_1_1_mixer_matrix. Make sure you are running this from the correct directory.")
    sys.exit(1)

# Non-blocking keyboard input is tricky in Python. 
# For this lab, we'll simulate "holding" keys by incrementing state.
class VirtualStick:
    def __init__(self):
        self.thrust = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
    def update(self):
        """
        Simulate a 'flight path' or let user adjust.
        For this demo, we'll oscillate to show the mixing.
        """
        # In a real version, we'd use 'keyboard' or 'curses' for input.
        # Here, we run a demo sequence.
        t = time.perf_counter()
        
        # Hover throttle (0.0 to 1.0)
        self.thrust = 0.5 + 0.2 * (0 if t % 10 < 5 else 0) # Base hover
        
        # Pitch oscillation (Sine wave)
        import math
        self.pitch = 0.2 * math.sin(t * 2)
        
        # Roll oscillation (Cosine wave)
        self.roll = 0.2 * math.cos(t * 1.5)
        
        # Yaw
        self.yaw = 0.0

def generate_table(stick, motors):
    table = Table(title="SQUID DYNO - Real-Time Mixer Analysis")
    
    table.add_column("Motor ID", style="cyan", no_wrap=True)
    table.add_column("Role", style="magenta")
    table.add_column("Value (0.0-1.0)", style="green")
    table.add_column("Output Power", style="yellow", width=40)

    # Motor Mapping (Standard Betaflight X)
    # M4 (CW)   M2 (CCW)
    #      \
    #       \
    #       /
    #      /
    # M3 (CCW)  M1 (CW)
    
    definitions = [
        ("M1", "Rear Right (CW)", motors[0]),
        ("M2", "Front Right (CCW)", motors[1]),
        ("M3", "Rear Left (CCW)", motors[2]),
        ("M4", "Front Left (CW)", motors[3]),
    ]

    for name, role, val in definitions:
        # Clamp value for visualization
        clamped_val = max(0.0, min(1.0, val))
        
        # Create a bar manually or use rich's built-in if preferred.
        # We'll use a manual ASCII/Block bar for control.
        bar_len = 30
        filled = int(clamped_val * bar_len)
        bar = "█" * filled + "░" * (bar_len - filled)
        
        # Color coding the value
        val_str = f"{val:.4f}"
        if val > 1.0 or val < 0.0:
            val_str = f"[bold red]{val_str}[/bold red] (SATURATED)"
        
        table.add_row(name, role, val_str, bar)

    return table

def main():
    console = Console()
    stick = VirtualStick()
    
    console.print("[bold green]Starting Virtual Dyno...[/bold green]")
    console.print("Simulating stick inputs to test mixer logic.")
    time.sleep(1)

    with Live(refresh_per_second=20) as live:
        try:
            while True:
                stick.update()
                
                # RUN THE MIXER (This is the code you wrote in Lab 1.1)
                motors = motor_mixer(stick.thrust, stick.roll, stick.pitch, stick.yaw)
                
                # Render
                table = generate_table(stick, motors)
                
                # Add Input Display
                input_panel = Panel(
                    f"Thrust: {stick.thrust:.2f} | Roll: {stick.roll:.2f} | Pitch: {stick.pitch:.2f} | Yaw: {stick.yaw:.2f}",
                    title="Pilot Inputs",
                    style="blue"
                )
                
                layout = Layout()
                layout.split_column(Layout(input_panel, size=3), Layout(table))
                
                live.update(layout)
                time.sleep(0.05)
                
        except KeyboardInterrupt:
            pass

if __name__ == "__main__":
    main()
