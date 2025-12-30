# Module 2: The Telemetry Stack
**"If you can't see it, you can't fix it."**

In Module 1, the drone was a black box. Now, we will make it transparent. You will build a real-time dashboard to visualize the drone's "thoughts" and a black-box recorder to analyze its failures.

---

## **2.1 Networking: UDP Sockets**

### **Objective**
Stream sensor data from the Drone (Pi Zero) to your Laptop (Ground Station) over Wi-Fi.

### **Theory**
*   **TCP:** "Did you get that? No? I'll resend." (Too slow for flight).
*   **UDP:** "Here's data! Here's data! Here's data!" (Fast, low latency).
We use UDP. If a packet drops, we don't care. The next one is coming in 20ms.

### **Lab Procedure**
1.  **Drone Side (`src/comms/transmitter.py`):**
    *   Pack data into a JSON string or binary struct.
    *   Send via `socket.sendto()` to your Laptop's IP.
2.  **Laptop Side (`ground_station/receiver.py`):**
    *   Bind a socket to `0.0.0.0` and a specific port (e.g., 5005).
    *   Loop and print received data.

### **Deliverable**
*   A script that streams the Lidar distance from Module 1 to your laptop terminal at 50Hz.

---

## **2.2 The Dashboard**

### **Objective**
Text is hard to read at 50Hz. We need graphs.

### **Theory**
We need a way to visualize data.
*   **Simple:** Python `http.server` hosting a single `index.html` file with JavaScript.
*   **Advanced:** React.js + FastAPI (For when you want a professional Ground Control Station).

We will start with the **Simple** approach.

### **Lab Procedure**
1.  **The Relay:** Write `ground_station/relay.py`.
    *   Listen for UDP packets from the drone.
    *   Forward them to the browser using `python-socketio` or a simple WebSocket library.
2.  **The UI (`index.html`):**
    *   Include **Chart.js** via CDN.
    *   Connect to your Python Relay via WebSocket.
    *   **The Graph:** Use a rolling buffer (array of length 100) to plot the last 2 seconds of Lidar data.

### **Deliverable**
*   A live line chart on your laptop screen showing the distance as you move your hand over the drone.

---

## **2.3 The Logger**

### **Objective**
Wi-Fi is not perfect. It cuts out. We need onboard logging to debug crashes that happen when the dashboard is frozen.

### **Theory**
Writing to the SD card is slow (blocking I/O). If we write every loop, the drone will stutter.
**Solution:** The Buffer.
Store 1000 lines in RAM, then dump to disk in one chunk (or use a separate thread).

### **Lab Procedure**
1.  Create `src/utils/logger.py`.
2.  Format: `Timestamp, Lidar_Dist, Motor_Out, Loop_Time`.
3.  Implement a `flush()` method that writes the buffer to a CSV file.
4.  Call `flush()` only when the drone is Disarmed (safe) or use a background thread.

### **Deliverable**
*   A `.csv` file generated after a run. Open it in Excel/Pandas and plot the data.

---

## **Check: The Flight Recorder**
**Forensics 101.**

1.  Run the "Reflex" logic from Module 1.
2.  Wave your hand rapidly to stress the sensor.
3.  Stop the code.
4.  Pull the CSV logs.
5.  **Analysis:**
    *   Plot `Timestamp` vs `Lidar_Dist`.
    *   Calculate the **Standard Deviation** of the loop time. Is it stable?
    *   Did the motor command lag behind the sensor reading? (Calculate the delay).

**Submission:** A screenshot of your plot and the calculated "System Latency" (ms).
