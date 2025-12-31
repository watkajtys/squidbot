[Return to Course Map](../../../../COURSE_MAP.md)

# Study Guide 5: Control Theory
**Module 5: The Math of Flight**

### Critical Takeaways
1.  **The PID Loop:** Proportional (P) corrects for the present error, Integral (I) corrects for past accumulated error (steady-state error), and Derivative (D) predicts future error by measuring the rate of change.
2.  **Damping Ratios:** An underdamped drone overshoots and bounces. An overdamped drone is sluggish and takes too long to reach the target. A "Critically Damped" drone returns to the setpoint in the minimum possible time with zero overshoot.
3.  **Stability Margins:** Tuning is not just about performance; it is about robustness. A drone tuned "to the edge" might fly perfectly in still air but oscillate violently or crash when it encounters a gust of wind or a voltage drop.
4.  **Anti-Windup:** Integral error can grow to infinity if the drone is physically stuck (e.g., on a branch). Anti-windup logic is a critical safety feature that prevents the drone from "exploding" with power once it is freed.

### The Evolution of Control
*   **The Foundation (PID):** This is what you are building in `lab_5_pid.py`. It is reactive, looking only at the current error. It has been the backbone of flight since the 1920s and remains the gold standard for low-level motor control.
*   **The Industry Standard (MPC):** Modern drones (like those from DJI or Skydio) use **Model Predictive Control**. Unlike PID, MPC uses a physics model to "predict" the future and solves an optimization problem at every step to find the best path.
*   **The Frontier (HJ Reachability):** The newest standard for high-stakes autonomy. It calculates a "Safe Set" of every possible future state to mathematically guarantee the drone never enters a state from which recovery is physically impossible (the "Unavoidable Collision" set).

### Mental Models and Field Notes
*   **The Tug of War:** Tuning is a constant trade-off between **Responsiveness** (how fast it moves) and **Robustness** (how stable it stays). If you make the drone faster (High Gain), it will eventually start to resonate and vibrate itself to death.
*   **The "Stick-to-Finger" Feeling:** A well-tuned drone should feel like it is physically connected to your transmitter sticks by a stiff iron rod. If it feels like it is connected by a rubber band or a spring, your Gains are too low.
*   **The Error Horizon:** A PID controller is essentially a "Time Machine." It looks at the **Past** (Integral), the **Present** (Proportional), and the **Future** (Derivative) to decide what to do right now.
*   **The Safety Shield:** Think of HJ Reachability like an "Invisible Forcefield" that isn't just a circle around the drone, but a circle in *Time*. It knows that if you are flying 10m/s toward a wall, you are already "dead" 5 meters before you hit it, because your braking distance is 6 meters.
*   **From Reactive to Proactive:** Moving from PID to MPC/HJ is like moving from "Driving by looking only at the front bumper" to "Driving by looking 500 meters down the road." You stop reacting to bumps and start planning to avoid them.

### Frontier Facts and Historical Context
*   **1892: The Lyapunov Proof:** While we use it for drones today, Aleksandr Lyapunov originally developed his stability theory to understand the movement of celestial bodies. His math proves that if you can find a "Scalar Energy Function" that always decreases, your system will eventually reach a stable state.
*   **Morphing Airframes:** Current research is moving beyond rigid drones. Scientists are building "Morphing" drones with wings that can fold mid-flight like a bird. These drones require "Non-Linear Adaptive Control" because their mass and inertia values literally change while they are in the air.
*   **Reinforcement Learning Control:** Traditional PID control is being challenged by AI. Instead of hand-tuning P, I, and D values, we let an AI fly the drone millions of times in simulation. The AI often discovers "Non-Linear" control strategies that humans would never think of, such as using the drone's own motor torque to "flip" through narrow gaps faster than a PID loop ever could.

---

### The Squid Games: Level 5
**The Statue Challenge**
Hover the drone autonomously. It must stay within a 20cm "cube" of space for 30 consecutive seconds without human intervention.
*   **The Goal:** Minimize your "RMS Error"—the average distance from the center of the cube.
*   **Win Condition:** A "Statue" hover that looks like the drone is hanging from an invisible string. 
*   **Elite Upgrade (The Bowling Ball):** Roll an object across the floor; the drone must use its **MPC** logic to weave around it and return to its spot without slowing down.

---

### Module 5 Quiz
1.  **Proportional Gain:** What happens to the "Rise Time" and "Steady-State Error" as you increase the P-gain?
2.  **Derivative Kick:** Why do we usually calculate the Derivative term based on the "Measurement" change rather than the "Error" change?
3.  **Feed-Forward:** In a drone, we know the motors must produce a force equal to $mg$ (gravity) to hover. How do we use this "prior knowledge" in our control law to help the PID loop?
4.  **Lyapunov Stability:** What is a "Lyapunov Function," and why do we use it to prove that a drone won't spin out of control?

---
*Reference: Lecture 5 (Control Theory) in docs/LECTURES.md*