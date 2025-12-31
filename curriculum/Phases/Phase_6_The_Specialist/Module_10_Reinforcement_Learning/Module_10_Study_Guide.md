[Return to Course Map](../../../../COURSE_MAP.md)

# Study Guide 10: Reinforcement Learning
**Module 10: Learning to Fly**

### Critical Takeaways
1.  **Policy vs. Classical Control:** Unlike a PID controller which is hand-tuned by an engineer, a Reinforcement Learning (RL) "Policy" is a neural network that learns via trial and error. The drone receives a **Reward ($R$)** for desired behaviors and a **Penalty** for undesired ones.
2.  **The Sim-to-Real Gap:** This is the primary hurdle in robotic RL. A policy trained in a "Perfect" simulation will often fail in the real world due to unmodeled physics. We combat this using **Domain Randomization**—randomly varying gravity, mass, and friction during training.
3.  **Proximal Policy Optimization (PPO):** PPO is the gold-standard algorithm for robotics. It uses a "Clipped" objective function to ensure that the policy doesn't change too drastically in a single training step. This stability is critical for physical maneuvers.
4.  **Observation Space ($S$):** The intelligence of the drone is limited by what it can "see." A robust observation space typically includes the current state, the history of previous actions (to account for latency), and the "Goal" vector.

### The Evolution of Intelligence
*   **The Foundation (Heuristics):** Early drones used "If/Then" logic (e.g., "If Obstacle < 1m, turn left"). While predictable, this logic fails in complex, "noisy" real-world environments like forests or crowded rooms.
*   **The Industry Standard (Deep RL):** This is what you explore in `lab_10_6_sim_to_real.py`. We use **Curriculum Learning** to teach the drone complex tasks by slowly increasing the difficulty—starting with a large target and gradually shrinking it until it is the size of a window.
*   **The Frontier (Liquid Neural Networks):** The newest standard for edge AI. Unlike static networks, LNNs use differential equations that allow the neurons to "adapt" their time-constants in real-time. This allows a drone to maintain performance even if its physical properties change.

### Mental Models and Field Notes
*   **The Cyber-Dog:** Training an RL agent is like training a dog. If you give the dog a treat for "Sitting," it learns to sit. If you accidentally give it a treat while it is "Barking and Sitting," it will learn to bark before it sits. This is called **Reward Aliasing**.
*   **The Black Box Problem:** A Neural Network is a "Black Box." If it decides to flip the drone upside down, it is often difficult to understand *why*. This is why RL is typically used for high-level "Tactics" while classical PID handles the low-level "Safety."
*   **Exploration vs. Exploitation:** During training, the agent must occasionally take a "Random" action to see if it leads to a better reward (Exploration). Once it finds a good path, it should stick to it (Exploitation).

### Frontier Facts and Historical Context
*   **The "Billion-Iteration" Pilot:** An RL agent can "fly" for the equivalent of 100 years in a single weekend of simulation. This is how AI pilots like those in the DARPA ACE program learn to perform maneuvers that would kill a human pilot through G-force or disorientation.
*   **Liquid Neural Networks (LNNs):** Unlike traditional NNs, which have fixed weights after training, LNNs are based on continuous-time differential equations. They can "adapt" their behavior on-the-fly to changing environment conditions (like a sudden drop in temperature affecting battery voltage). For a low-power device like the Pi Zero, LNNs require significantly fewer neurons to achieve complex behavior.
*   **Curriculum Learning:** How do you teach a drone to fly through a window? You don't start with a window. You start by rewarding it for flying in a straight line. Then, you place a massive 10-meter hole in its way. Slowly, you shrink the hole until it is the size of a window.

---

### The Squid Games: Level 10
**The Sim-to-Real Resilience Challenge**
Using the pre-trained model in `lab_10_6_sim_to_real.py`, evaluate the policy's performance as you manually increase the "Sensor Noise" and "Action Latency" in the simulator.
*   **The Goal:** Identify the "Breaking Point"—the exact amount of latency (in milliseconds) where the RL policy becomes unstable.
*   **Win Condition:** Successfully modifying the model's reward function in the script to prioritize "Smoothness" over "Speed," and proving that the new model survives higher latency.

---

### Module 10 Quiz
1.  **Markov Decision Process (MDP):** Define the four components of an MDP $(S, A, P, R)$.
2.  **Discount Factor ($\gamma$):** Why do we use a discount factor in RL? What happens if $\gamma = 0$?
3.  **Policy Gradients:** Explain the "Log-Derivative Trick" used in policy gradient algorithms.
4.  **Reward Engineering:** You are training a drone to hover. You give it a reward for "Staying near $(0,0,1)$." The drone learns to stay near $(0,0,1)$ by vibrating at 500Hz. How would you change your reward function to prevent this?

---
*Reference: Lecture 10.1 (Reinforcement Learning Foundations) in docs/LECTURES.md*