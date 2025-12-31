[Previous Module](../Module_13_VIO/Module_13_Lecture.md) | [Course Map](../../../../COURSE_MAP.md) | [Next Module](../Module_15_Deep_Perception/Module_15_Lecture.md)

---

# Module 14: Swarm Theory
**"The Intelligence of the Crowd."**

In this final module, you move from one drone to a collective. We will explore how to make multiple drones behave like a single organism.

---

## **14.1 Graph Theory: The Skeleton of the Swarm**

### **The Laplacian Matrix**
How do we mathematically describe "Neighborly" behavior? We represent the swarm as a **Graph** $G = (V, E)$.
*   **The Adjacency Matrix ($A$):** $A_{ij} = 1$ if drone $i$ can hear drone $j$.
*   **The Laplacian ($L$):** $L = D - A$.
*   **The Consensus Equation:** The simplest way to make robots agree is the protocol $\dot{x} = -Lx$.
*   **The Fiedler Value ($\lambda_2$):** The second smallest eigenvalue of $L$. It measures how well the swarm is connected. If $\lambda_2 = 0$, the swarm has split into two separate groups.

### **14.1.1 Sub-Lab: Graph Connectivity**
1.  **Launch:** 5 drones in simulation.
2.  **Distance-Based Mesh:** Set the Wi-Fi range to $5\text{m}$. 
3.  **The Test:** Drag one drone away until it's $>5\text{m}$ from everyone else.
    *   **Observation:** The $\lambda_2$ value drops to 0. The consensus fails. The isolated drone "drifts" because it has no neighbors to pull it back.

---

## **14.2 Distributed Formation Control**

### **The Shape Vector**
Drones don't just want to be at the same spot; they want to form a Triangle or a Circle.
*   **Target Displacement ($\Delta_{ij}$):** Each drone is told "I want to be $1\text{m}$ East of Drone 2."
*   **The Law:** $\dot{x}_i = -\sum (x_i - x_j - \Delta_{ij})$.
*   **Emergent Behavior:** Even if you only tell Drone 1 about the waypoint, the "Tension" in the graph will pull the entire formation along, like a group of people holding hands.

### **14.2.1 Event-Triggered Communication: Ghost Models**
Wi-Fi is a shared resource. If 10 drones talk at $100\text{Hz}$, the network crashes.
*   **The Shadow Model:** Each drone runs a local simulation of its neighbors. 
*   **The Error Threshold ($\epsilon$):** A drone only shouts its "Real" position if it deviates from its own Shadow prediction by more than $10\text{cm}$.
*   **The Result:** During steady flight, the Wi-Fi stays silent. During a collision-avoidance maneuver, the Wi-Fi "bursts" with high-frequency data.

### **14.2.2 Just-In-Time Math: The Sidewalk Dance (RVO)**
**"Don't Stop, Just Slide"**

In `lab_7_3_rvo_collision.py`, we implement a "Nudge" logic.
*   **The Problem:** If two drones head-on, both might stop (Deadlock).
*   **The Solution (RVO):** "I assume you will move 50% Right. I will move 50% Right."
*   **The Math:** We calculate the **Tangent Vector** (perpendicular to the collision). We project our velocity onto this tangent to "slide" past the obstacle without losing momentum.

**AI Prompt:** "I have two 2D points and velocities. Calculate the 'Time to Closest Approach' (tau). If tau is positive and small, calculate a velocity correction vector to avoid collision."

---

## **14.3 Socratic Discussion: The Collective Mind**
1.  **Question:** In a swarm of 100 drones, what happens if 10 drones are "hacked" and start pulling the group toward a wall?
    *   **Answer:** This is the **Byzantine Fault** problem. We use **Resilient Consensus** (filtering out the "Outlier" neighbors) to ensure the $90\text{%}$ majority can mathematically ignore the $10\text{%}$ defectors.
2.  **Question:** Why do birds fly in a 'V' shape?
    *   **Answer:** Aerodynamics (Module 0). The leader creates a "Vortex" that provides lift to the bird behind it. In autonomous swarms, we must model this "Prop Wash" so that the Laplacian feedback doesn't fight the physical airflow.

---

## **Check: The Synchronized Dance**
**The Graduation Ceremony.**

1.  **Scenario:** 3 drones in a line.
2.  **Task:** Perform a "Figure 8" where the drones must cross paths without colliding.
3.  **Constraint:** You can only control the "Swarm Center." The individual drone movements must be handled by the Laplacian math.

**Submission:** A simulation video showing the drones weaving through each other safely.

---
## **Theoretical Foundations**

### Lecture 14: Swarm Theory & Graph Consensus

#### **1. Laplacian Graph Dynamics**
How do multiple agents coordinate without a centralized controller? We treat the swarm as a **Graph** $G = (V, E)$.
*   **The Laplacian Matrix ($L$):** Defined as $L = D - A$, where $D$ is the Degree matrix and $A$ is the Adjacency matrix.
*   **The Consensus Protocol:** Each drone follows the update law $\dot{x}_i = -\sum_{j \in N_i} (x_i - x_j)$. In matrix form, this is $\dot{x} = -Lx$.
*   **Convergence:** The swarm will converge to a single position if and only if the graph has a spanning tree. The **Algebraic Connectivity** (the second smallest eigenvalue of $L$, $\lambda_2$) determines the "speed" at which the swarm reaches agreement.

#### **2. Control Barrier Functions (CBF) & Safety**
A PID controller is a "Goal Seeker." It doesn't care if there is a wall in the way. A CBF is a "Guardian."
*   **The Safe Set ($\mathcal{C}$):** We define a scalar function $h(x) \geq 0$ (e.g., $h = \text{distance} - 0.2\text{m}$).
*   **The QP Filter:** We take the desired control $u_{pid}$ and solve a **Quadratic Program**:
    $$\min_u ||u - u_{pid}||^2 \text{ s.t. } \dot{h}(x, u) \geq -\alpha(h(x))$$
*   **Provable Safety:** This ensures that as the drone approaches a wall ($h \to 0$), the CBF mathematically overrides the PID command, forcing the drone to "slide" along the boundary without ever crossing it.

#### **3. Event-Triggered Communication**
Constant peer-to-peer messaging destroys Wi-Fi bandwidth.
*   **The Trigger:** A drone only broadcasts its state if its current position $x(t)$ deviates from its last broadcast position $x(t_k)$ by more than a threshold $\epsilon$.
*   **The Ghost Model:** Every drone runs a local simulation of its neighbors. It only requests a "Real update" when the neighbor's real physics departs from the simulated "Ghost" belief.

**Next Step:** [Module 15: Deep Perception](../Module_15_Deep_Perception/Module_15_Lecture.md)

---

## **Professor's Brief: Speaker Notes**

### **The Narrative Hook**
"We are moving from an 'Individual' to a 'Collective.' A single bird can be caught by a hawk. A swarm of birds becomes a confusing, shifting organism that can survive. Today, we write the rules of consensus. We aren't just controlling robots; we are engineering intelligence from the bottom up. We are teaching our drones how to 'think' as a single mind."

### **Deep Research Context: Switching Topologies**
In PhD research, we must account for **Switching Topologies**. Drones move; they go behind walls and lose connection to their neighbors. The graph $G$ changes every second. Explain that the swarm remains stable as long as the "Integral of Connectivity" is sufficient—even if the drones are disconnected $90\text{%}$ of the time, as long as they "glimpse" each other occasionally, they will still maintain formation.

## **Mastery Checklist**
Before moving to the next module, you should be able to:
- [ ] Diagram the Adjacency and Laplacian matrices for a 3-drone formation.
- [ ] Explain how the Fiedler Value ($\lambda_2$) determines the stability of a swarm.
- [ ] Define the mathematical relationship between a PID command and a CBF "Safety Filter."
- [ ] Describe the "Ghost Model" approach to reducing network congestion in swarms.

---

## **Further Reading & Bibliography**

### **Consensus Theory**
*   **Olfati-Saber, R., & Murray, R. M. (2004).** *"Consensus problems in networks of agents with directed graphs and switching topology."* IEEE Transactions on Automatic Control.
*   **Mesbahi, M., & Egerstedt, M. (2010).** *Graph Theoretic Methods in Multi-Agent Systems.* Princeton University Press. (The definitive textbook).

### **Safety Proofs**
*   **Ames, A. D., et al. (2019).** *"Control Barrier Functions: Theory and Applications."* IEEE European Control Conference.

---

[Previous Module](../Module_13_VIO/Module_13_Lecture.md) | [Course Map](../../../../COURSE_MAP.md) | [Next Module](../Module_15_Deep_Perception/Module_15_Lecture.md)