# Mini-Lecture 0.1: Programming Robotic Objects
**"Why we use Classes for Hardware."**

In simple Python, you might write `print("Hello")`. In Robotics, we use **Object-Oriented Programming (OOP)**. 

---

## **1. The "Digital Twin" Concept**
Imagine your VL53L1X Lidar sensor. It has a state (is it on?) and a behavior (tell me the distance).
Instead of writing scattered variables, we wrap them in a **Class**.

```python
class LidarSensor:
    def __init__(self, address):
        self.address = address
        self.is_running = False

    def start(self):
        # Hardware specific code to turn it on
        self.is_running = True

    def get_distance(self):
        if self.is_running:
            return 1.2 # Placeholder for real hardware reading
        return None
```

---

## **2. Why this matters for the Squid Project**
As your drone grows, you will have dozens of sensors.
*   By using classes, your `main.py` can look like this:
    ```python
    drone = SquidDrone()
    dist = drone.lidar.get_distance()
    if dist < 0.2:
        drone.motors.stop()
    ```
*   This is called **Abstraction**. You don't need to remember the I2C hex codes for the Lidar every time you want a measurement; you just ask the `lidar` object.

---

## **3. `self` is the Robot**
The most confusing part of Python OOP is the `self` keyword.
*   `self` refers to the specific physical hardware instance you are talking to.
*   If you have two Lidars (Up and Down), `self` ensures that `up_lidar.read()` doesn't accidentally return the data from `down_lidar`.

**Exercise:** Look at `src/drivers/tof_array.py`. Notice how it uses a class to manage two sensors at once.
--- [Return to Course Map](../../../COURSE_MAP.md)