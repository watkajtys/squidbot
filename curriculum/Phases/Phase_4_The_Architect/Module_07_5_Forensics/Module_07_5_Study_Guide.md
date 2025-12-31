# Study Guide 7.5: The Forensic Regression Suite
**Module 7.5: The Architect (System Verification and CI/CD)**

### Critical Takeaways
1.  **Code is a Liability:** Every line of code added to a flight controller is a potential point of failure. **Regression Testing** is the only way to ensure that "fixing" a bug in the EKF doesn't accidentally introduce a drift that crashes the drone during a high-speed maneuver.
2.  **The Golden Bag is the Truth:** We treat historical flight data (MCAP/Rosbags) as the "Ground Truth." In a perfect system, replaying the same sensor data through the same code should result in the same physical estimate. If it doesn't, your system has "Non-Determinism," which is a death sentence for robotics.
3.  **RMSE as a Scorecard:** We don't guess if the new code is "better." We quantify it using **Root Mean Square Error**. If the RMSE between the new estimate and the historical "Success" estimate is too high, the code is rejected, even if it "looks fine" in a bench test.

### Mental Models and Field Notes
*   **The Time Machine:** Regression testing allows you to "Go back in time" to the exact moment a sensor failed or a controller oscillated. You can tweak a single variable and see exactly how the outcome would have changed without ever risking a battery or a prop.
*   **Shadow Execution:** Imagine two brains in one drone. One brain (The Veteran) is actually flying. The other brain (The Rookie) is just listening and thinking. Post-flight, we compare the Rookie's "Imaginary" flight to the Veteran's "Actual" flight. If they diverge, the Rookie isn't ready for a solo mission.
*   **Smug vs. Paranoid Filters:** In Forensic analysis of an EKF, we look at the **Residuals**. If the residuals are much smaller than the filter's estimated covariance, the filter is "Smug" (overconfident). If they are much larger, the filter is "Paranoid" (it doesn't trust its own sensors).

### Frontier Facts and Historical Context
*   **SpaceX "Shadow Mode":** Before SpaceX flies a new Starship landing algorithm, they run it in shadow mode on a Falcon 9. The new code thinks it's landing, but it's actually just logging data while the old code does the work.
*   **Differentiable Simulators:** The next generation of forensics uses simulators like **NVIDIA Isaac Sim** or **Brax** where the entire physics engine is "Differentiable." This means you can ask the computer: "What should I change in my code to make this crash NOT happen?" and the computer can calculate the exact mathematical answer.
*   **Consistency Tests (NIS):** The **Normalized Innovation Squared (NIS)** test is the "Polygraph" for Kalman Filters. It uses a Chi-Squared distribution to mathematically prove if a filter's internal logic matches the external reality.

---

### The Squid Games: Level 7.5
**The Saboteur Challenge**
Run the `tools/regression_test.py` script against a `GoldenBag.mcap`.
*   **The Goal:** Intentionally change the `EKF_ACCEL_NOISE` parameter in your code and observe the RMSE spike. 
*   **Win Condition:** Identifying the exact timestamp where the "Modified" estimate diverged from the "Golden" estimate by more than 5cm.

---

### Module 7.5 Quiz
1.  **RMSE:** You have two sets of altitude data. Set A has a constant 2cm offset. Set B has a single 20cm spike but is otherwise perfect. Which one will likely have a higher RMSE?
2.  **Determinism:** List three things in a Python script that can cause "Non-Deterministic" behavior (Hint: Consider system time, threading, and random seeds).
3.  **Shadowing:** Why is Shadow Mode safer than testing code in a simulator? (Hint: Consider the "Reality Gap").
4.  **Forensics:** You observe that the EKF's Innovation (Residual) is consistently positive. What does this tell you about your sensor's calibration (e.g., the Accelerometer Z-bias)?

---
*Reference: Lecture 7.5 (Forensics) in curriculum/Phases/Phase_4_The_Architect/Module_07_5_Forensics/Module_07_5_Lecture.md*
