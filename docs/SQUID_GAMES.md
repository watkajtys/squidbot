# The Squid Games: Skill Challenges
**"Prove it. Record it. Share it."**

To prevent "Theory Burnout," use these challenges as your milestones. Don't move to the next Phase until you've "Won" the game for the current one.

---

## Phase I: The "Heartbeat" Game
*   **The Challenge:** "The Morse Code." 
*   **Goal:** Write a script that takes a string (e.g., "SQUID") and blinks the onboard LED in perfect Morse Code. 
*   **The Win:** Seeing the hardware respond to your logic for the first time.

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

## Phase VI: The "Combat" Game
*   **The Challenge:** "The Red Rover."
*   **Goal:** Use the camera to find a red ball. The drone must "Lock On" and maintain exactly 1 meter distance, even if you move the ball.
*   **The Win:** A "Fox Two" lock-on where the drone follows you like a loyal pet.

## Phase VII: The Frontier (Experimental)

*   **The Challenge:** "The Synchronized Dance."

*   **Goal:** With 3 drones, make them all rotate in a circle while maintaining a perfect triangle formation.

*   **The Win:** Pure Swarm Intelligence.

*   **Elite Upgrade:** "The Silent Hunt." Briefly disconnect the Wi-Fi; the drones must use **Event-Triggered Ghost Models** to maintain formation in the silence.



---



# 🏁 THE CAPSTONE: "The Search & Rescue Mission"

**This is the final exam. There is no guide. There is only the objective.**



### **The Scenario**

A "Survivor" (an AprilTag) is hidden somewhere in an unknown, multi-room environment. The environment contains obstacles, low-light zones, and a "No-Fly Zone" (a red-taped area).



### **The Mission Requirements**

1.  **Autonomous Takeoff:** No manual input allowed.

2.  **Exploration:** The drone must autonomously map the environment using **SLAM**.

3.  **Hazard Avoidance:** Must not enter the "No-Fly Zone" or hit any walls.

4.  **Target Acquisition:** Locate the AprilTag survivor using the camera.

5.  **Precision Perch:** Once the survivor is found, find the nearest "Perch" (flat surface > 1m high) and land on it to monitor the scene.

6.  **Telemetry:** Provide a full **Post-Mortem MCAP Log** and a **Foxglove Dashboard** video of the entire flight.



### **The Reward**

Completion of this mission marks you as a **Squid Systems Architect**. You have mastered the stack from the soldering iron to swarm intelligence.
