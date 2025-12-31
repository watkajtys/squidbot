# Study Guide 14: Swarm Theory
**Module 14: Consensus and Coordination**

### Critical Takeaways
1.  **Algebraic Graph Theory:** A swarm of drones is mathematically modeled as a **Graph ($G$)**. The drones are "Nodes" and their communication links are "Edges." We use the **Laplacian Matrix ($L$)** of this graph to analyze the swarm's behavior. The second smallest eigenvalue of the Laplacian (the **Fiedler Value**) determines how fast the swarm can reach a "Consensus" on its state.
2.  **The Consensus Protocol:** To fly in formation without a central leader, each drone follows a simple local rule: $\dot{x}_i = \sum (x_j - x_i)$ for all neighbors $j$. This rule ensures that all drones eventually converge to the same position, heading, or velocity, using only local peer-to-peer communication.
3.  **Control Barrier Functions (CBF):** Safety in a swarm is guaranteed by CBFs. A CBF defines a "Safe Set" (e.g., $Dist > 0.5m$) and ensures that the drone's control input always satisfies a condition that keeps the system within that set. If a high-level "Formation" command would cause a collision, the CBF logic will "filter" the command to maintain safety.
4.  **Distributed Task Allocation:** How does a swarm decide which drone should go to which target? We use **Market-Based Algorithms** (like the Auction Algorithm). Each drone "bids" on a task based on its distance and battery level. Through a distributed negotiation, the tasks are assigned to the most efficient drones without a central commander.

### The Evolution of Coordination
*   **The Foundation (Centralized Control):** The "Old Tech" method where one "Ground Station" tells every drone exactly where to go. While simple to implement, it creates a single point of failure—if the ground station crashes, the entire swarm crashes.
*   **The Industry Standard (Decentralized Consensus):** This is the focus of `lab_14_4_multi_agent_trap.py`. Every drone is an independent thinker. They talk to their neighbors and "agree" on a formation. This makes the swarm incredibly resilient to the loss of any single drone.
*   **The Frontier (Bio-Inspired Emergence):** The newest standard, mimicking the behavior of birds or bees. These swarms don't even use specific "Consensus" math; they use simple, reactive rules like "Separation, Alignment, and Cohesion" to perform complex maneuvers at high speeds.

### Mental Models and Field Notes
*   **The Elastic Net:** Think of a swarm as a group of drones connected by invisible rubber bands. If one drone moves, it "pulls" its neighbors with it. Consensus is the process of the rubber bands reaching a state of minimum tension.
*   **The Safety Bubble:** Imagine every drone is surrounded by a pressurized "Air Bubble." As two drones get closer, the pressure increases until they are physically pushed apart. A Control Barrier Function is the mathematical implementation of this pressure.
*   **The Intelligence of the Crowd:** A single ant is simple, but an ant colony can find the shortest path to food. In a swarm, "Intelligence" is an emergent property. No single drone knows the "Master Plan," but through simple local interactions, the group can perform complex maneuvers.

### Frontier Facts and Historical Context
*   **Communication-Aware Control:** Standard swarms assume the Wi-Fi always works. "Comm-Aware" swarms treat the radio signal as a physical constraint. If a drone is about to fly behind a thick concrete wall that would break the consensus link, the control logic will automatically pull it back into "Signal Range."
*   **The "Silent Hunt":** In the Squid Games, we test "Event-Triggered Ghost Models." If the Wi-Fi briefly disconnects, the drones use an internal physics simulation of their neighbors to "guess" where they are, maintaining formation in total silence.
*   **Safety Barriers and Swarm Collisions:** In 2018, a swarm of 1,374 drones in Xi'an, China, suffered a massive failure when GPS interference caused the drones to lose consensus. If those drones had been using **Control Barrier Functions**, they would have at least avoided colliding with each other.

---

### The Squid Games: Level 14
**The Synchronized Dance Challenge**
Using `lab_14_4_multi_agent_trap.py`, coordinate three simulated drones to maintain a "Triangle Formation" while moving toward a goal.
*   **The Goal:** Reach the goal without the triangle "collapsing" or the drones colliding.
*   **Win Condition:** Formation error < 0.1 meters throughout the entire flight.

---

### Module 14 Quiz
1.  **Graph Theory:** If a swarm's communication graph is "Disconnected," can the entire group reach a consensus?
2.  **The Fiedler Value:** If you add more communication links between drones, does the Fiedler Value increase or decrease? Does the swarm reach consensus faster?
3.  **Collision Avoidance:** Explain the difference between "Potential Fields" and "Control Barrier Functions."
4.  **Consensus:** How do you modify the consensus protocol to make the drones maintain a fixed *offset* instead of all meeting at the same point?

---
*Reference: Lecture 14 (Swarm Theory) in docs/LECTURES.md*
