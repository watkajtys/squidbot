# Module 7.6: The Forensic Regression Suite
**"Trust, but Verify."**

In professional robotics (Tesla, SpaceX, Waymo), you are not allowed to "just fly" new code. Even a 1% change in a PID gain or an EKF covariance could cause a crash in an edge case you didn't test. We use **Regression Testing** to ensure that new code doesn't break old success.

---

## **7.6.1 The "Golden Bag" (Reference Data)**

### **Objective**
Create a "Perfect" reference dataset.

### **Lab Procedure**
1.  **Flight:** Perform a 30-second "Statue Hover" (Module 5).
2.  **Record:** Save the flight as a ROS 2 `.mcap` file (The "Golden Bag").
3.  **The Oracle:** This bag contains the "Truth." Any future version of your code should be able to process the sensors in this bag and arrive at the same conclusion as your original code.

---

## **7.6.2 Headless Execution (The Simulator)**

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

## **7.6.3 The "Diff" (Identifying Regressions)**

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
