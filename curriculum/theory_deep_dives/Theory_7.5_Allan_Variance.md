# Theory Deep Dive 7.5: Allan Variance & Noise
**"Measuring the Silence."**

In your EKF (Module 7), you have a variable `R` (Sensor Noise). How do you find that number? You don't guess. You use **Allan Variance**.

---

## **1. The Three Types of Noise**
1.  **White Noise (Angle Random Walk):** High-frequency "jitter." (Easy to filter).
2.  **Bias Instability:** The "center" of the sensor drifts over minutes. (Hard to filter).
3.  **Rate Random Walk:** The drift itself starts drifting. (The "PhD" level of noise).

---

## **2. The Allan Deviation (ADEV) Plot**
By plotting the variance of your sensor over different time scales, you get a "U-shaped" curve.
*   **The Left side:** Tells you the White Noise.
*   **The Bottom of the U:** Tells you the Bias Instability.

**Lab Task:** 
1. Let your drone sit perfectly still for 1 hour while logging IMU data.
2. Run the `lab_7_5_allan_variance.py` script.
3. Use the resulting numbers as the input for your EKF. **This is why research drones are more stable than hobby ones.**
