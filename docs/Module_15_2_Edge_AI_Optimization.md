# Module 15.2: Edge AI & Model Optimization
**"Making the AI fit the Pi."**

A standard Deep Learning model (like a ResNet or a large CNN) is designed for a PC with a massive GPU. On the Raspberry Pi Zero, it is too slow. If your AI takes 500ms to "see" a wall, your drone (traveling at 2m/s) has already crashed.

---

## **15.2.1 The "Inference" Budget**

### **Objective**
Understand the cost of a forward pass.

### **Theory**
*   **FLOPs (Floating Point Operations):** The number of math steps needed for one "Thought."
*   **Memory Bandwidth:** The Pi Zero has limited RAM (512MB). Large models will cause "Swap" and kill performance.
*   **Goal:** We need our perception model to run in **< 33ms** (30Hz) using only 20% of the CPU.

---

## **15.2.2 Quantization: FP32 to INT8**

### **Objective**
Shrink the weights without losing the "Brain."

### **Theory**
*   **FP32 (32-bit):** Standard "high-precision" math.
*   **INT8 (8-bit):** "Low-precision" math. 
*   **The Magic:** By converting weights from 32-bit to 8-bit, you shrink the model size by **4x** and often speed up execution by **2-3x** with only a 1% drop in accuracy.

### **Lab Procedure**
1.  **The Tool:** Use **TensorFlow Lite (TFLite)** or **ONNX Runtime**.
2.  **Conversion:**
    ```python
    import tensorflow as tf
    converter = tf.lite.TFLiteConverter.from_keras_model(my_model)
    converter.optimizations = [tf.lite.Optimize.DEFAULT]
    tflite_model = converter.convert()
    ```
3.  **Benchmark:** Compare the `inference_time` of the raw model vs the TFLite model on the Pi.

---

## **15.2.3 Model Pruning & Bottlenecks**

### **Objective**
Cut the "Dead Wood."

### **Theory**
*   **Pruning:** Removing "Neurons" that have near-zero weights. They aren't contributing to the decision, so why do the math?
*   **MobileNet Architecture:** Instead of standard Convolutions, use **Depthwise Separable Convolutions**. This is the "Gold Standard" for mobile robotics.

### **Lab Procedure**
1.  **Architecture Swap:** Replace your Module 15.1 CNN with a `MobileNetV2` backbone.
2.  **Validation:** Retrain the model on the "Survivor" dataset.
3.  **Efficiency:** Measure the "Frames Per Second" (FPS) on the Pi.

---

## **The "Real-Time AI" Test**
**The Silicon Challenge.**

1.  Start your **Autonomous Mapping** node (Module 8).
2.  Start your **TFLite Perception** node (Module 15.2).
3.  **The Test:** Can you maintain a steady **50Hz Control Loop** while the AI is running at **30Hz**?
    *   **Fail:** If the control loop drops to 20Hz, your AI is "stealing" too much CPU. You must prune the model further.

**Submission:** A table comparing Model Size (MB), Accuracy (%), and Inference Time (ms) for your original vs optimized models.
