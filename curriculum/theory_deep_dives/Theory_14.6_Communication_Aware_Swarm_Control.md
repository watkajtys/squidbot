# Theory Deep Dive 14.6: Comm-Aware Control
**"Fighting in the Silence."**

In a dogfight, your radio link is a liability. If your swarm coordination depends on a perfect 100Hz Wi-Fi link, a single "Lag Spike" will cause a mid-air collision.

---

## **1. Event-Triggered Control (ETC)**
In standard control, we send state updates every 10ms.
*   **The ETC Way:** We only send an update if $\| x_{actual} - x_{last\_sent} \| > \text{Threshold}$.
*   **Result:** We reduce Wi-Fi traffic by 80%, leaving more "Air Time" for critical collision avoidance packets.

---

## **2. Consensus under Time-Delay**
If Drone A sends a position, and Drone B receives it 200ms later, Drone B is looking at a "Ghost."
*   **The Fix:** Every packet must have a **Timestamp**. 
*   **Prediction:** Drone B uses its internal physics model to "Fast Forward" Drone A's position from the timestamp to the current time.

---

## **3. Graceful Degradation**
If communication fails entirely, the swarm must switch from "Cooperative" (Auction) to "Non-Cooperative" (Independent Pursuit) without crashing. This is why we use **RVO Avoidance** (Module 14.5) as a safety layer—it works even if the drones can't talk.
