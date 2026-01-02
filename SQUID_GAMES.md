# The Squid Games: Skill Challenges
**"Prove it. Record it. Share it."**

To prevent "Theory Burnout," use these challenges as your milestones. Don't move to the next Phase until you've "Won" the game for the current one.

---

## Phase I: The "Nervous System" Games
**Module 0 Challenge: "The Heartbeat"**
*   **Goal:** Write a script that takes a string (e.g., "SQUID") and blinks the onboard LED in perfect Morse Code. 
*   **The Win:** Seeing the hardware respond to your logic for the first time.

**Module 1 Challenge: "The Limp" (System Failure)**
*   **Goal:** Simulate a motor failure in your `lab_1_1_mixer_matrix.py` logic. Write a "Limp Mode" mixer that maintains a level hover (Roll/Pitch = 0) using only 3 motors.
*   **The Math:** You must redistribute the torque load. If Motor 1 dies, Motor 3 (its diagonal) must reduce power to balance Yaw, while Motors 2 & 4 increase power to balance Thrust.
*   **The Win:** Running the Virtual Dyno (`lab_1_5`) and seeing the bars re-balance automatically when you trigger the failure flag.

## Phase II: The "Horizon" Game
*   **The Challenge:** "The Virtual Level."
*   **Goal:** Using your HUD from Module 3, tilt the drone in your hand. The "Virtual Horizon" must stay perfectly level with the actual floor.
*   **The Win:** A video of you moving the drone while the AR HUD stays rock-solid.

## Phase III: The "Statue" Game
*   **The Challenge:** "The Coffee Cup."
*   **Goal:** Hover the drone. It must stay within a 20cm "cube" of space for 30 seconds. 
*   **The Win:** A "Statue" hover that looks like the drone is hanging from an invisible string.
*   **Elite Upgrade:** "The Bowling Ball." Roll an object across the floor; the drone must use **MPC** to weave around it without slowing down.

## Phase IV: The "Outlier" Game
*   **The Challenge:** "The Paper Trick."
*   **Goal:** While the drone is estimating its height via EKF, slide a piece of paper under the Lidar and pull it away. 
*   **The Win:** The drone's "estimated" altitude stays steady because your EKF realized the paper was an "outlier."
*   **Elite Upgrade:** "The Jink Master." Track a target being shaken erratically. Your **IMM Filter** must keep the lock-on reticle rock-solid.

## Phase V: The "Ghost" Game
*   **The Challenge:** "The Mirror Maze."
*   **Goal:** Fly the drone in a room with a mirror.
*   **The Win:** Your SLAM algorithm (Module 8) correctly identifies the mirror and doesn't try to fly into the "Ghost Room."
*   **Elite Upgrade:** "The Raven." Find a high perch (shelf/cabinet) above 1.5m and land on it autonomously using your **Perch Finder** logic.

## Phase VI: The "Tactical" Game
*   **The Challenge:** "The Red Rover."
*   **Goal:** Use the camera to find a red ball. The drone must "Lock On" and maintain exactly 1 meter distance, even if you move the ball.
*   **The Win:** A "Fox Two" lock-on where the drone follows you like a loyal pet.
*   **Integrated Project:** [The Ghost in the Machine](PROJECTS.md#project-3-the-ghost-in-the-machine-tactical-autonomy). Intercept a virtual enemy while respecting physical safety boundaries.

## Phase VII: The Frontier "Game"
*   **The Challenge:** "The Synchronized Dance."
*   **Goal:** With 3 drones (or 1 real + 2 virtual), make them rotate in a circle while maintaining a perfect formation.
*   **The Win:** Pure Swarm Intelligence.
*   **Elite Upgrade:** "The Silent Hunt." Briefly disconnect the Wi-Fi; the drone must use **Event-Triggered Ghost Models** to maintain its trajectory in the silence.
*   **Integrated Project:** [The Silent Guardian](PROJECTS.md#project-2-the-silent-guardian-stealth--stability). Maintain a rock-solid hover and adapt gains using **Liquid Neural Networks**.

---

# THE CAPSTONES: Final Graduation Missions
**Refer to:** [FINAL_CHALLENGE.md](FINAL_CHALLENGE.md)

This is the final exam. There is no guide. There is only the objective. Choose your path:

*   **Path A: The Tactical Architect (Operation Deep Ink)** - Focus on robust control and estimation in GPS-denied zones.
*   **Path B: The Swarm Commander (Operation Hive Mind)** - Focus on consensus, formation, and multi-agent coordination.
*   **Path C: The AI Researcher (Operation Neural Nest)** - Focus on Reinforcement Learning, LNNs, and Sim-to-Real transfer.

### **The Reward**
Completion of any mission marks you as a **Squid Systems Architect**. You have mastered the stack from the soldering iron to swarm intelligence.