[Previous Module](../Module_07_State_Estimation/Module_07_Lecture.md) | [Course Map](../../../../COURSE_MAP.md) | [Next Module](../../Phase_5_The_Researcher/Module_08_Perception_and_Mapping/Module_08_Lecture.md)

---

# Module 07.5: The Forensic Regression Suite
**"Trust, but Verify."**

In professional robotics (Tesla, SpaceX, Waymo), you are not allowed to "just fly" new code. Even a 1% change in a PID gain or an EKF covariance could cause a crash in an edge case you didn't test. We use **Regression Testing** to ensure that new code doesn't break old success.

---

## **07.5.1 The "Golden Bag" (Reference Data)**

### **Objective**
Create a "Perfect" reference dataset.

### **Lab Procedure**
1.  **Flight:** Perform a 30-second "Statue Hover" (Module 5).
2.  **Record:** Save the flight as a ROS 2 `.mcap` file (The "Golden Bag").
3.  **The Oracle:** This bag contains the "Truth." Any future version of your code should be able to process the sensors in this bag and arrive at the same conclusion as your original code.

---

## **07.5.2 Headless Execution (The Simulator)**

### **Objective**
Run your code "Offline" against the data.

### **Theory**
We will write a Python script that:
1.  Opens the `.mcap` file.
2.  Feeds the sensor messages into your `EKFNode` or `PIDNode` **one by one**.
3.  Records what your code *would* have done.

### **Lab Procedure**
1.  **Code:** Create `tools/regression_test.py`.
2.  **Logic:**
    *   Load the `GoldenBag.mcap`.
    *   Initialize your `EKF` class (without the ROS 2 wrapper, just the raw math).
    *   For each sensor packet: `output = my_ekf.update(packet)`.
    *   Save the output to a CSV.

---

## **07.5.3 The "Diff" (Identifying Regressions)**

### **Objective**
Quantify the difference between "Old" and "New" code.

### **Lab Procedure**
1.  **The Comparison:** Plot the `Original_Altitude` (from the bag) against your `New_Altitude` (from the test script).
2.  **The Metric (RMSE):** Calculate the **Root Mean Square Error**.
    $$RMSE = \sqrt{\frac{1}{N} \sum (Alt_{new} - Alt_{old})^2}$$
3.  **The "Fail" Condition:** If $RMSE > 0.01$ (1cm), your code has "Regressed." You must explain *why* the math changed before you are allowed to fly.

---

## **The "Forensic" Challenge**
**"The Sabotage."**

1.  I will give you a "Golden Bag" of a perfect flight.
2.  I will also give you a "Sabotaged" version of the EKF code where I have subtly changed one parameter (e.g., increased IMU noise).
3.  **Your Job:** Run the regression suite. Identify exactly which timestamp the code diverged and use the **Residual Plot** to prove that the code is no longer "Consistent" with the reality of the bag.

**Submission:** A plot of the "Regression Delta" and the calculated RMSE.

---
## **Theoretical Foundations**

### Lecture 7.5: CI/CD for Robotics & Forensic Analysis

#### **1. Differentiable Robotics**
Modern research often treats the entire drone as a differentiable function. Regression testing is essentially checking the **Gradient** of the system. If a small change in code leads to a massive change in behavior, your system is "Ill-conditioned."

#### **2. Consistency Tests (NIS/NEES)**
How do we know if our EKF is "Healthy"? 
*   **Normalized Innovation Squared (NIS):** We check if the difference between the sensor and our guess matches the noise we expected. 
*   **Chi-Squared Test:** If the NIS value stays within a certain bound, the filter is "Consistent." If it drifts out, the filter is "Smug" (it thinks it's better than it is) or "Paranoid."

---

## **Professor's Brief: Speaker Notes**

### **The Narrative Hook**
"When a rocket explodes on the launchpad, engineers don't guess why. They look at the black box. They replay the sensor data frame-by-frame through their simulations. This is 'Robotic Forensics.' Today, you are building the time machine that allows you to replay reality until you find the truth."

### **Deep Research Context**
*   **Deterministic Replay:** In Python, things are usually deterministic, but in C++ or parallel systems, they are not. Explain that to have a true regression suite, you must eliminate 'Race Conditions' where the order of sensor arrival changes the outcome.
*   **Shadowing:** In industry, we often run 'Shadow Code' on the robot. The real code flies the drone, but the 'Experimental' code runs in the background, logging what it *would* have done. We then compare the two post-flight.

## **Mastery Checklist**
Before moving to the next module, you should be able to:
- [ ] Define the "Golden Bag" principle for deterministic replay.
- [ ] Calculate RMSE (Root Mean Square Error) between two flight datasets.
- [ ] Explain the benefit of "Shadow Execution" for testing experimental controllers.
- [ ] Perform a Residual Plot analysis to identify EKF inconsistency.

---

## **Further Reading & Bibliography**

### **System Testing**
*   **Koopman, P. (2010).** *Better Embedded System Software.* Dr. Phil's Press. (Best practices for regression).
*   **Bar-Shalom, Y., et al. (2001).** *Estimation with Applications to Tracking and Navigation.* Wiley. (The source for NIS/NEES consistency tests).

---

[Previous Module](../Module_07_State_Estimation/Module_07_Lecture.md) | [Course Map](../../../../COURSE_MAP.md) | [Next Module](../../Phase_5_The_Researcher/Module_08_Perception_and_Mapping/Module_08_Lecture.md)