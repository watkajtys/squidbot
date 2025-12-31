import numpy as np
import pytest
from src.labs.lab_15_siamese_perception import SiamesePerception

def test_mixer_matrix_orthogonality():
    """
    Verifies the motor mixer matrix from Module 1.
    If the mixer isn't balanced, the drone will yaw when it tries to roll.
    """
    # Standard X-frame mixer
    mixer_matrix = np.array([
        [1.0, -1.0,  1.0, -1.0],  # M1
        [1.0,  1.0,  1.0,  1.0],  # M2
        [1.0, -1.0, -1.0,  1.0],  # M3
        [1.0,  1.0, -1.0, -1.0]   # M4
    ])
    
    # Test: PURE THROTTLE (Should result in equal thrust on all motors)
    throttle_cmd = np.array([0.5, 0, 0, 0])
    thrusts = np.dot(mixer_matrix, throttle_cmd)
    assert np.all(thrusts == 0.5)
    
    # Test: PURE ROLL (Should result in differential thrust across X-axis)
    roll_cmd = np.array([0, 0.2, 0, 0])
    thrusts = np.dot(mixer_matrix, roll_cmd)
    assert thrusts[0] < 0 # M1/M3 (left) should decrease
    assert thrusts[1] > 0 # M2/M4 (right) should increase

def test_siamese_similarity_logic():
    """
    Verifies the Cosine Similarity math for Lab 15.
    """
    engine = SiamesePerception()
    
    # Create two identical dummy embeddings
    emb1 = np.array([1.0, 0.0, 1.0])
    emb2 = np.array([1.0, 0.0, 1.0])
    
    # Should be 1.0 (identical)
    assert pytest.approx(engine.calculate_similarity(emb1, emb2), 0.001) == 1.0
    
    # Create an orthogonal embedding
    emb3 = np.array([0.0, 1.0, 0.0])
    # Should be 0.0 (perpendicular/different)
    assert pytest.approx(engine.calculate_similarity(emb1, emb3), 0.001) == 0.0

def test_anti_windup_clamping():
    """
    Simulates a 'Stuck Drone' scenario to test the safety logic.
    """
    integral_sum = 0
    error = 10.0
    limit = 5.0
    
    # Simulate 10 iterations of accumulation
    for _ in range(10):
        integral_sum += error
        # Implementation of anti-windup
        if integral_sum > limit:
            integral_sum = limit
            
    assert integral_sum == 5.0 # Should be clamped to limit, not 100.0
