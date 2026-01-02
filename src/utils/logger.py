"""
Lab 1.4: The Silent Observer (Async Logger)

Goal: Capture flight data without blocking the main control loop.

THEORY: The Cost of Observation
-------------------------------
In Quantum Mechanics, observing a particle changes its state. In Robotics, logging data changes your timing.
Writing to a file (SD Card) is a "Blocking Operation." The Operating System (Linux) halts your program 
while it waits for the physical write head (or flash controller) to accept the data.

On a Raspberry Pi Zero 2 W, a simple `file.write()` can take anywhere from 1ms to 50ms depending on 
system load and disk buffer state.
- Control Loop Target: 50Hz (20ms period).
- If `write()` takes 25ms, you have missed your deadline. The drone will stutter.

SOLUTION: The Producer-Consumer Pattern
---------------------------------------
We decouple "Generation" (Fast) from "Storage" (Slow) using a Queue and a Background Thread.
1. Main Thread (Producer): Puts data into a Queue (RAM operation, ~100 nanoseconds).
2. Queue (Buffer): Holds the data temporarily.
3. Worker Thread (Consumer): Pulls data from Queue and writes to Disk (Slow operation).

Result: The Main Thread never waits for the Disk.
"""

import csv
import time
import threading
import queue
from datetime import datetime

class AsyncLogger:
    def __init__(self, filename=None):
        """
        Initialize the Logger.
        
        Args:
            filename (str): Path to the log file. If None, generates a timestamped name.
        """
        if filename is None:
            # ISO 8601-ish format: YYYYMMDD_HHMMSS
            filename = f"flight_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        
        self.filename = filename
        
        # The Queue is thread-safe. It acts as the buffer between the fast flight loop
        # and the slow disk writer. If the writer falls behind, the queue grows in RAM.
        self.queue = queue.Queue()
        
        # Flag to control the background thread lifecycle
        self.running = True
        
        # Define headers based on what we expect to log
        # "loop_dt" is critical for measuring Jitter (See Module 00 Deep Dive)
        self.headers = [
            "timestamp", 
            "loop_dt",      # Time since last loop iteration (should be ~0.02s)
            "roll", "pitch", "yaw",
            "m1", "m2", "m3", "m4"
        ]
        
        # Initialize the background thread
        # daemon=True means this thread will be killed automatically if the main program exits.
        # This prevents the script from hanging if we crash before calling close().
        self.thread = threading.Thread(target=self._writer_worker, daemon=True)
        self.thread.start()
        print(f"[Logger] Background thread started. Writing to: {self.filename}")

    def log(self, data):
        """
        The Fast Path (Called from the Flight Loop).
        
        This method must return INSTANTLY. No disk I/O allowed here.
        
        Args:
            data (dict): Dictionary containing sensor values matching self.headers.
        """
        # Auto-inject timestamp if missing (using high-precision performance counter)
        if "timestamp" not in data:
            data["timestamp"] = time.perf_counter()
            
        # Put data in the queue.
        # block=False means "If the queue is full, crash immediately rather than waiting."
        # Ideally, we'd handle the Full exception, but for now, we want to know if we are overflowing.
        self.queue.put(data)

    def _writer_worker(self):
        """
        The Slow Path (Runs in Background Thread).
        
        This loop is isolated from the Flight Controller. It can block, stall, 
        or sleep without affecting the drone's stability.
        """
        print("[Logger] Worker thread active.")
        
        # Open file once and keep it open. Opening/Closing files is expensive.
        with open(self.filename, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=self.headers)
            writer.writeheader()
            
            # Keep running as long as the main thread says so, OR if there is still left-over data in the queue.
            while self.running or not self.queue.empty():
                try:
                    # Get data from queue.
                    # timeout=1.0 allows the loop to check 'self.running' periodically 
                    # even if no data is coming in.
                    data = self.queue.get(timeout=1.0)
                    
                    # This is the blocking call!
                    writer.writerow(data)
                    
                    # Mark task as done (useful if we were using queue.join())
                    self.queue.task_done()
                    
                except queue.Empty:
                    # No data arrived in the last 1.0 seconds. 
                    # Loop back to check if self.running is still True.
                    continue
                except Exception as e:
                    print(f"[Logger Error] Write failed: {e}")

    def close(self):
        """
        Graceful Shutdown.
        
        We must ensure all data currently in the Queue is written to disk before we exit.
        """
        print("[Logger] Shutting down...")
        self.running = False # Signal the worker thread to stop accepting new work
        self.thread.join()   # Wait for the worker thread to finish flushing the queue
        print(f"[Logger] Log closed safely. Saved {self.filename}")