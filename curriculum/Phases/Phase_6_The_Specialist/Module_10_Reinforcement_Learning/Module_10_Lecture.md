[Previous Module](../../Phase_5_The_Researcher/Module_09_Trajectory_Optimization/Module_09_Lecture.md) | [Course Map](../../../../COURSE_MAP.md) | [Next Module](../Module_11_Tactics_and_Guidance/Module_11_Lecture.md)

---

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

---
## **Theoretical Foundations**

### Lecture 10.1: Reinforcement Learning & MDPs

#### **1. Markov Decision Processes (MDP)**
We model the drone's flight as an MDP defined by $(S, A, P, R, \gamma)$.
*   **State ($S$):** The 12D state vector (Position, Velocity, Orientation, Angular Rate).
*   **Transition Probability ($P$):** The probability $P(s_{k+1} | s_k, a_k)$, which encapsulates the drone's physics.
*   **The Reward Gradient:** We seek a Policy $\pi(a|s)$ that maximizes the Expected Return $J(\pi) = \mathbb{E}_{\pi} [\sum_{t=0}^{\infty} \gamma^t r_t]$.

#### **2. Proximal Policy Optimization (PPO)**
PPO is the industry standard for robust robotics.
*   **The Trust Region:** Traditional Policy Gradient methods can fail if the policy update is too large. PPO uses a **Clipped Surrogate Objective**: 
    $L^{CLIP}(\theta) = \hat{\mathbb{E}}_t [\min(r_t(\theta)\hat{A}_t, \text{clip}(r_t(\theta), 1-\epsilon, 1+\epsilon)\hat{A}_t)]
*   **The Advantage:** This ensures that the update never deviates too far from the previous successful policy, preventing the "Catastrophic Forgetting" that causes drones to fall out of the sky mid-training.

#### **3. Liquid Neural Networks (LNNs)**
For real-world adaptation (e.g., a motor getting hot), we utilize LNNs.
*   **Continuous-Time Dynamics:** Unlike standard RNNs, LNNs are defined by differential equations: $\frac{dx}{dt} = -A(s)x + B(s)u$. 
*   **Time-Constant Learning:** The "Liquid" property means the network learns its own **Time Constants**. It can slow down its thinking during steady flight and speed up during high-frequency disturbances, allowing for sub-millisecond adaptation to physical damage.

**Next Step:** [Module 11: Aerial Combat & Guidance](../Module_11_Tactics_and_Guidance/Module_11_Lecture.md)

---

## **Professor's Brief: Speaker Notes**

### **The Narrative Hook**
"We are moving from 'Hand-Crafted' logic to 'Learned' logic. In previous modules, you were the engineer. In this module, you are the parent. You don't tell the drone how to fly; you tell it what makes you happy (the Reward Function), and you let it fail thousands of times in the simulator until it discovers the physical laws of flight on its own."

### **Deep Research Context: Reward Hacking**
In PhD-level RL, the biggest risk is **Reward Hacking**. If you reward 'Staying Alive' but don't penalize 'Spinning in Circles,' the AI will find a mathematically valid way to get the reward that violates the intent of the mission. Mention that we use **Curriculum Learning**: we start with a simple task (Hover), and once the AI achieves $90\text{%}$ success, we "increase the gravity" or "add wind" to force the policy to become more robust.

## **Mastery Checklist**
Before moving to the next module, you should be able to:
- [ ] Define the Action and Observation spaces for a drone in a Gym environment.
- [ ] Explain how the Clipping function in PPO prevents catastrophic policy collapse.
- [ ] Describe the goal of Domain Randomization in bridging the Sim-to-Real gap.
- [ ] List the primary mathematical difference between an RNN and a Liquid Neural Network (LNN).

---

## **Further Reading & Bibliography**

### **Policy Gradients**
*   **Schulman, J., et al. (2017).** *"Proximal Policy Optimization Algorithms."* arXiv:1707.06347. (The foundational PPO paper).
*   **Sutton, R. S., & Barto, A. G. (2018).** *Reinforcement Learning: An Introduction.* MIT Press.

### **Robotics AI**
*   **Hasani, R., et al. (2021).** *"Liquid Time-constant Networks."* AAAI Conference on Artificial Intelligence. (The definitive LNN paper).
*   **Hwangbo, J., et al. (2019).** *"Learning agile and dynamic motor skills for legged robots."* Science Robotics.

---

[Previous Module](../../Phase_5_The_Researcher/Module_09_Trajectory_Optimization/Module_09_Lecture.md) | [Course Map](../../../../COURSE_MAP.md) | [Next Module](../Module_11_Tactics_and_Guidance/Module_11_Lecture.md)