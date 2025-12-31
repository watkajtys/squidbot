# Module 10: Reinforcement Learning (The Brain)
**"Code that writes itself."**

Classical Control (PID, Splines) is "Explicit Programming." You tell the drone exactly how to fly.
Reinforcement Learning (RL) is "Implicit Programming." You tell the drone *what you want* (Reward), and it figures out *how* to do it (Policy).

---

## **10.1 The Simulation (Gym)**

### **Objective**
Train a pilot in the Matrix.

### **Theory**
*   **Agent:** The Drone.
*   **Environment:** PyBullet Physics.
*   **Action Space:** 4 Motor RPMs (Normalized -1 to 1).
*   **Observation Space:** IMU (Rotation/Rate) + Target Vector.
*   **Reward Function:**
    *   $+1.0$ for staying alive.
    *   $-0.1 * DistanceToTarget$.
    *   $-100$ for crashing.

### **Lab Procedure**
1.  **Install:** `pip install gym-pybullet-drones stable-baselines3`.
2.  **The Environment:** Create a custom Gym Env `HoverAviary`.
3.  **The Algorithm:** Use **PPO (Proximal Policy Optimization)**. It is robust and standard for robotics.
### **10.1.1 Sub-Lab: Reward Hacking**
**"The Lazy AI."**

AI is inherently lazy. It will find the "path of least resistance" to get a reward, even if it's not what you intended.

1.  **Experiment:** Modify your `HoverAviary` environment to have a "Broken" reward:
    `reward = 1.0` (Fixed reward for every frame alive).
2.  **Train:** Run for 100,000 steps.
3.  **Observe:** The drone won't learn to hover. It will likely learn to **Spin at maximum RPM** or "Vibrate" on the ground to avoid crashing. 
4.  **The Lesson:** This is **Reward Hacking**. If you don't penalize "Energy Usage" or "Distance from Target," the AI will find a "degenerate" solution.

---

## **10.2 Sim-to-Real (The Gap)**

### **Objective**
Make it fly in the real world.

### **Theory**
A Neural Network trained in a perfect simulator will fail in reality because:
1.  **Latency:** Real motors react slower.
2.  **Noise:** Real IMUs vibrate.
3.  **Aerodynamics:** Prop wash is chaotic.

**Solution: Domain Randomization.**
During training, we randomly change:
*   Drone Mass (+/- 10%).
*   Motor Friction.
*   Sensor Noise.
*   Latency (10ms - 30ms).
This forces the AI to learn a "Robust Policy" that works *everywhere*.

---

## **10.3 Deployment**

### **Objective**
Run the `.zip` model on the Pi Zero.

### **Lab Procedure**
1.  **Export:** Convert the PyTorch model to **ONNX** (Open Neural Network Exchange). ONNX runs much faster on CPU.
2.  **Inference Engine:** Write `src/ai/pilot.py` using `onnxruntime`.
3.  **The Loop:**
    *   Read IMU.
    *   Run ONNX Model.
    *   Send Motor Commands.
    *   (Must run at >50Hz).

---

## **Check: The Uncrashable Drone**
**The Push Test 2.0.**

1.  **Launch:** The RL Agent takes off.
2.  **Disturb:** Push it. Throw a pillow at it.
3.  **Observe:**
    *   PID Controllers react *after* the error happens.
    *   RL Agents often react *proactively* or recover from extreme angles (inverted) that would confuse a PID controller.

**Submission:** A video comparing your PID Hover vs. your RL Hover in strong wind (fan).
