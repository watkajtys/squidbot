# Python Robotics Patterns: The Rosetta Stone
**"Translating Whiteboards into Code."**

This guide shows you how to translate common robotics mathematics into clean Python code using `NumPy`. Use this as a reference when the Syllabus asks you to "Implement the math."

---

## **1. Linear Algebra (Matrices & Vectors)**
In textbooks, you see: $\mathbf{y} = \mathbf{A} \mathbf{x} + \mathbf{b}$

```python
import numpy as np

# Define a 2x2 Matrix (A)
A = np.array([[1.0, 2.0], 
              [3.0, 4.0]])

# Define a Vector (x)
x = np.array([5.0, 6.0])

# Matrix Multiplication (Ax)
# Use the @ operator or np.dot()
y = A @ x + np.array([1.0, 1.0])
```

---

## **2. Calculus (Discrete Time)**
In textbooks, you see: $v = \frac{dx}{dt}$ (Derivative) or $pos = \int v \, dt$ (Integral).
In a robot, we don't have infinite time; we have "Ticks" (loops).

### **The Derivative (Rate of Change)**
How fast is my sensor changing?
```python
# Save the previous value and time
delta_time = current_time - last_time
derivative = (current_value - last_value) / delta_time
```

### **The Integral (Accumulation)**
How much error has built up over time?
```python
# Accumulate the value multiplied by time
integral += current_value * delta_time
```

---

## **3. Geometry & Rotations**
**NEVER** write your own rotation math (Sine/Cosine) for 3D space. It leads to Gimbal Lock and misery. Use `scipy`.

```python
from scipy.spatial.transform import Rotation as R

# To rotate a vector:
r = R.from_euler('xyz', [45, 0, 0], degrees=True)
vector_body = [0, 0, 1]
vector_world = r.apply(vector_body)
```

---

## **4. Frequency (The FFT)**
When the labs ask you to "Find the vibration frequency":

```python
import numpy as np

# Perform Fast Fourier Transform
# 'data' is a list of Gyro readings
fft_result = np.fft.fft(data)
frequencies = np.fft.fftfreq(len(data), d=1/sample_rate)

# Find the peak (The loudest vibration)
peak_idx = np.argmax(np.abs(fft_result))
vibration_hz = abs(frequencies[peak_idx])
```

---

## **5. Logic: The "Watchdog" Pattern**
Safety is a math problem. If the time since the last packet is too high, the drone is "dead."

```python
if (time.time() - last_heartbeat_time) > SAFETY_THRESHOLD:
    emergency_disarm()
```
--- [Return to Course Map](../../../COURSE_MAP.md)