[Previous Module](../Module_15_Deep_Perception/Module_15_Lecture.md) | [Course Map](../../../../COURSE_MAP.md) | [Next: Final Challenge](../../../../FINAL_CHALLENGE.md)

---

# Module 15.5: Edge AI & Model Optimization
**"Making the AI fit the Pi."**

A standard Deep Learning model (like a ResNet or a large CNN) is designed for a PC with a massive GPU. On the Raspberry Pi Zero, it is too slow. If your AI takes 500ms to "see" a wall, your drone (traveling at 2m/s) has already crashed.

---

## **15.5.1 The "Inference" Budget**

### **Objective**
Understand the cost of a forward pass.

### **Theory**
*   **FLOPs (Floating Point Operations):** The number of math steps needed for one "Thought."
*   **Memory Bandwidth:** The Pi Zero has limited RAM (512MB). Large models will cause "Swap" and kill performance.
*   **Goal:** We need our perception model to run in **< 33ms** (30Hz) using only 20% of the CPU.

---

## **15.5.2 Quantization: FP32 to INT8**

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

## **15.5.3 Model Pruning & Bottlenecks**

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
2.  Start your **TFLite Perception** node (Module 15.5).
3.  **The Test:** Can you maintain a steady **50Hz Control Loop** while the AI is running at **30Hz**?
    *   **Fail:** If the control loop drops to 20Hz, your AI is "stealing" too much CPU. You must prune the model further.

**Submission:** A table comparing Model Size (MB), Accuracy (%), and Inference Time (ms) for your original vs optimized models.

---
## **Theoretical Foundations**

### Lecture 15.5: Numerical Efficiency & On-Device AI

#### **1. The von Neumann Bottleneck in Robotics**
On an embedded system like the Pi Zero, the bottleneck is often **Memory Access** rather than CPU cycles.
*   **The Problem:** Large weight matrices cannot fit in the L1/L2 cache. The CPU wastes thousands of cycles waiting for data to arrive from RAM.
*   **The Fix:** We utilize **Model Pruning** and **Weight Clustering** to reduce the number of unique values in the model, allowing for better compression and cache hits.

#### **2. Linear Quantization Math**
We convert 32-bit real numbers ($r$) to 8-bit integers ($q$) using an affine transformation.
*   **The Formula:** $q = \text{clip}(\text{round}(\frac{r}{S} + Z), q_{min}, q_{max})$. 
*   **Scale ($S$):** The step size of the quantization.
*   **Zero-point ($Z$):** The integer value that represents the real-world zero.
*   **Integer-Only Inference:** By performing the entire forward pass in INT8 math, we bypass the floating-point unit (FPU) entirely, saving power and increasing throughput on the Pi's ARM core.

#### **3. MobileNet & Depthwise Separable Convolutions**
A standard convolution filters pixels and colors simultaneously. MobileNet splits this into two steps:
1.  **Depthwise:** One filter per input channel ($3\times 3 \times 1$).
2.  **Pointwise:** A $1\times 1 \times C$ convolution to combine the results.
*   **FLOPs Reduction:** This reduces the computational cost by a factor of roughly $\frac{1}{N} + \frac{1}{D_k^2}$. For a $3\times 3$ filter, the AI becomes **$8\text{--}9\text{x}$ faster** with minimal accuracy loss.

---

## **Professor's Brief: Speaker Notes**

### **The Narrative Hook**
"Today we are performing 'Brain Surgery' on our AI. We are removing $90\text{%}$ of its neurons and forcing it to think in 8-bit integers instead of complex decimals. We are teaching the drone to be 'Smart Enough' to fly, but 'Simple Enough' to survive on a $\$15$ processor. This is the art of Efficiency Engineering."

### **Deep Research Context: The Pareto Frontier**
In PhD-level research, there is no "best" AI. There is only the **Pareto Frontier**. This is the curve representing models where you cannot improve accuracy without increasing latency. Explain that our goal is to "Push the Frontier" by finding architectures that offer better "Accuracy-per-Watt." Mention **NAS (Neural Architecture Search)**: using AI to design even smaller AI that fits perfectly inside the Pi's specific cache size.

## **Mastery Checklist**
Before moving to the next module, you should be able to:
- [ ] Explain the INT8 affine transformation formula used in model quantization.
- [ ] Describe the computational benefit of Depthwise Separable Convolutions.
- [ ] Identify the von Neumann Bottleneck and its impact on embedded AI.
- [ ] Define the Pareto Frontier for latency-accuracy trade-offs.

---

## **Further Reading & Bibliography**

### **Edge Optimization**
*   **Howard, A. G., et al. (2017).** *"MobileNets: Efficient Convolutional Neural Networks for Mobile Vision Applications."* arXiv:1704.04861.
*   **Jacob, B., et al. (2018).** *"Quantization and Training of Neural Networks for Efficient Integer-Arithmetic-Only Inference."* IEEE CVPR.

### **Neural Pruning**
*   **Han, S., et al. (2015).** *"Learning both Weights and Connections for Efficient Neural Network."* NeurIPS. (The seminal pruning paper).

---

[Previous Module](../Module_15_Deep_Perception/Module_15_Lecture.md) | [Course Map](../../../../COURSE_MAP.md) | [Next: Final Challenge](../../../../FINAL_CHALLENGE.md)