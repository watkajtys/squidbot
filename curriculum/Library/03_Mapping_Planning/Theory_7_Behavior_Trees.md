# Theory Deep Dive 7: Behavior Trees & Mission Logic
**"The Robot's Decision Engine."**

In Module 12, your drone needs to make complex decisions (Takeoff -> Search -> Land).

---

## **1. Why `if/else` is the Enemy**
Imagine this logic:
```python
if battery > 10:
    if target_found:
        approach()
    else:
        search()
else:
    land()
```
Now add: "What if the GPS signal is lost?" "What if a human overrides?" The code becomes a "Spaghetti" mess of nested `if` statements.

---

## **2. What is a Behavior Tree (BT)?**
A BT is a graphical tree of tasks. It "ticks" from the top down.

### **The Three Main Nodes:**
1.  **Action Nodes:** Do something (e.g., `SpinMotor`, `ReadCamera`).
2.  **Selector Nodes (?)**: "Try A. If A fails, try B." (Useful for Failsafes).
3.  **Sequence Nodes (->)**: "Do A. If A succeeds, then do B." (Useful for missions like Takeoff -> Fly).

---

## **3. The "Tick"**
The tree doesn't "run" once. It "ticks" at 10Hz. Every 0.1s, the tree re-evaluates the world.
*   *Benefit:* If you are in the middle of "Flying" and the battery suddenly goes "Low," the tree will immediately switch branches to "Land" on the next tick.

---

## **4. Integration (PyTrees / BehaviorTree.CPP)**
In ROS 2, we use standard libraries for this. You don't write the tree in `if` statements; you define it in an **XML file**. This allows you to change the drone's "personality" without changing a single line of Python code.

**Study Task:** Look up the "StarCraft 2 AI" or "Halo AI." They both use Behavior Trees to make their bots feel "intelligent."
--- [Return to Course Map](../../../COURSE_MAP.md)