[Return to Course Map](../../../../COURSE_MAP.md)

# Study Guide 11: Aerial Combat and Guidance
**Module 11: The Tactical Pilot**

### Critical Takeaways
1.  **Proportional Navigation (Pro-Nav):** Pro-Nav is the foundational guidance law for interception. It states that the commanded acceleration of the drone should be proportional to the **Line-of-Sight (LOS) Rate** to the target. Mathematically: $a_c = N \cdot V_c \cdot \dot{\lambda}$, where $N$ is the Navigation Constant (typically 3 to 5), $V_c$ is the Closing Velocity, and $\dot{\lambda}$ is the LOS rate.
2.  **Closing Geometry:** To intercept a moving target, a drone should not aim *at* the target, but rather at the **Predicted Intercept Point**. This requires solving the "Interception Triangle" in real-time, matching your drone's speed and heading to the target's future location.
3.  **Bayesian Search Theory:** When searching for an elusive target, we maintain a **Probability Map (Heatmap)**. As the drone's sensors scan the environment, we use **Bayes' Rule** to update the probability of each grid cell. If the sensor scans a cell and finds nothing, the probability of the target being there decreases, and the remaining probability redistributes to other cells.
4.  **Energy Maneuverability (E-M):** Aerial combat is a game of Energy Management. Potential Energy (Altitude) can be traded for Kinetic Energy (Speed) and vice versa. Every turn "bleeds" energy due to induced drag. A tactical drone must maintain its **Specific Excess Power ($P_s$)** to ensure it has enough energy to perform a final strike or an evasive maneuver.

### The Evolution of Guidance
*   **The Foundation (Pure Pursuit):** The simplest "Old Tech" method. The drone simply points its nose at the target and flies. While easy to implement, it results in a "tail-chase" that is inefficient and easily defeated by a turning target.
*   **The Industry Standard (Pro-Nav):** The "Leading" method. This is what you implement in `lab_11_pro_nav.py`. By aiming where the target *will be*, the drone maintains a constant bearing, which is the mathematically shortest path to interception.
*   **The Frontier (AI Dogfighting):** Using Reinforcement Learning to discover non-intuitive tactics. AI pilots can execute high-level tactical maneuvers like the "Yo-Yo" or "Split-S" with millisecond precision, outperforming human reaction times.

### Mental Models and Field Notes
*   **The Quarterback's Pass:** Intercepting a drone is exactly like a quarterback throwing a football to a running receiver. You don't throw to where the receiver is; you "lead" them by throwing to where they will be in 2 seconds.
*   **The Hunting Moth:** When being pursued, some insects use "Stochastic Evasion"—randomly changing their flight path. In robotics, we can implement this by adding a "Noise" component to our trajectory, making it mathematically impossible for an AI interceptor to predict our path.
*   **The Search for the Needle:** Searching is not just about "Looking." It is about "Excluding." Every empty room you scan makes the target's presence in the *unscanned* rooms more certain.

### Frontier Facts and Historical Context
*   **The "Fox Two" Lock-on:** In fighter jet terminology, "Fox Two" refers to an infrared-guided missile launch. In the Squid Project, your "Fox Two" is the camera-based tracking logic that keeps a red ball (or a target drone) in the center of the frame using Pro-Nav guidance.
*   **Moth-Inspired Evasion:** Moths use a "spiraling dive" to escape bats. We can implement a "Stochastic Evasion" lab where the drone uses a random-number generator to decide its next move, making it impossible for an AI to predict its path.
*   **Multi-Agent Dogfighting:** Recent DARPA research (ACE Program) demonstrated that AI pilots can defeat human fighter pilots in simulated 1-on-1 dogfights by using millisecond-perfect energy management and tactical prediction.

---

### The Squid Games: Level 11
**The Fox Two Challenge**
Using `lab_11_pro_nav.py`, intercept a target that is moving in a "Sine Wave" pattern.
*   **The Goal:** Tune your Navigation Constant ($N$) to minimize the "Miss Distance."
*   **Win Condition:** Miss distance < 0.2 meters over 10 consecutive trials.

---

### Module 11 Quiz
1.  **Pro-Nav:** Explain why a Navigation Constant $N=1$ is insufficient for interception.
2.  **Bayesian Search:** If your sensor has a 90% Probability of Detection ($P_d = 0.9$) and it scans a room and finds nothing, what is the new probability that the target is in that room?
3.  **E-M Theory:** Define "Corner Speed." Why is it the most efficient speed for a tactical turn?
4.  **Closing Velocity:** Why does the Pro-Nav command $a_c$ go to infinity as the distance to the target goes to zero if there is any LOS rate error?

---
*Reference: Lecture 12 (Differential Games) in docs/LECTURES.md*
