# Theory Deep Dive 15.5: Spatio-Temporal Change Detection
**"The Cartographer of Change."**

A robot that lives in a room should know when that room changes.

---

## **1. The Diff-Map**
We store two Octomaps:
*   **Map A (Monday):** The "Baseline."
*   **Map B (Tuesday):** The "Current Scan."
*   **Result:** `C = Map B - Map A`. Any voxels that remain are **New Objects** (e.g., a package on the floor or a moved chair).

---

## **2. Semantic Change**
By using your **Deep Perception (Module 15)**, you don't just see "something moved." You see "The *chair* moved." 
*   This allows the drone to generate a report: "The room is 95% consistent. 1 new object detected near the kitchen counter."

---

## **3. Tactical Use**
In a dogfight, Change Detection tells you where the enemy is hiding. If a "shadow" appears in a corner that was empty 10 seconds ago, that shadow is your target.
