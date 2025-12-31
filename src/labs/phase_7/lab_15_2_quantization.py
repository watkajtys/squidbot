"""
Lab 15.2: Edge AI Optimization (Quantization)
Goal: Convert a standard Keras/Torch model to TFLite Int8.
Refer to docs/Module_15_2_Edge_AI_Optimization.md

The Pi Zero 2 W lacks a GPU. We must use quantization to achieve 30Hz inference.
"""

class ModelQuantizer:
    def __init__(self, model_path):
        self.model_path = model_path

    def convert_to_int8(self):
        """
        Perform Post-Training Quantization (PTQ).
        """
        # TODO: Implement TFLite Converter logic
        # 1. Load the model
        # 2. Set representative dataset for calibration
        # 3. Set optimizations to [tf.lite.Optimize.DEFAULT]
        # 4. Enforce integer-only execution
        pass

    def benchmark(self, tflite_model):
        """
        Compare latency (ms) of FP32 vs INT8.
        """
        pass
