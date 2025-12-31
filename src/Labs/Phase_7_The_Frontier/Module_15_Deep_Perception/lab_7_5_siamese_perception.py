
SQUID DRONE CURRICULUM: lab_7_5_siamese_perception.py
----------------------------------
Context: ../../../../curriculum/Phases/Phase_7_The_Frontier/Module_15_Deep_Perception/Module_15_Lecture.md
Theory: curriculum/Library/
Win Condition: Phase VII: The AI Speed-Run (SQUID_GAMES.md)

This lab is part of the 'Soldering Iron to Swarm' masterclass.

import numpy as np
import cv2

class SiamesePerception:
    """
    Simulates a Siamese Network for Loop Closure Detection.
    In a real scenario, this would use a CNN (like MobileNet) 
    to extract 'embeddings' from images.
    """
    def __init__(self):
        # In a real lab, we'd load a PyTorch/TensorFlow model here.
        print("Initializing Metric Learning Engine (Simulated)...")

    def extract_embedding(self, image):
        """
        Extracts a feature vector (embedding) from an image.
        For this lab, we use a simple Histogram of Oriented Gradients (HOG)
        as a placeholder for a Deep Neural Network embedding.
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        resized = cv2.resize(gray, (64, 128))
        
        # Simple descriptor as a placeholder for CNN features
        winSize = (64, 128)
        blockSize = (16, 16)
        blockStride = (8, 8)
        cellSize = (8, 8)
        nbins = 9
        hog = cv2.HOGDescriptor(winSize, blockSize, blockStride, cellSize, nbins)
        embedding = hog.compute(resized)
        
        return embedding.flatten()

    def calculate_similarity(self, emb1, emb2):
        """
        Calculates the Cosine Similarity between two embeddings.
        1.0 = Identical, 0.0 = Completely Different.
        """
        dot_product = np.dot(emb1, emb2)
        norm_a = np.linalg.norm(emb1)
        norm_b = np.linalg.norm(emb2)
        return dot_product / (norm_a * norm_b)

    def is_loop_closure(self, current_img, database_imgs, threshold=0.85):
        """
        Checks if the current image matches any image in the database.
        """
        curr_emb = self.extract_embedding(current_img)
        
        max_similarity = -1.0
        match_idx = -1

        for i, db_img in enumerate(database_imgs):
            db_emb = self.extract_embedding(db_img)
            sim = self.calculate_similarity(curr_emb, db_emb)
            
            if sim > max_similarity:
                max_similarity = sim
                match_idx = i

        if max_similarity > threshold:
            return True, match_idx, max_similarity
        return False, -1, max_similarity

if __name__ == "__main__":
    print("Squid Drone: Lab 15 - Deep Perception & Loop Closure")
    
    engine = SiamesePerception()
    
    # Simulation: Load two similar and one different image
    # In practice, students would use frames from their drone's camera.
    print("\n--- Testing Similarity Engine ---")
    print("Algorithm: Cosine Similarity on Feature Embeddings")
    print("Goal: Recognize a 'Visited' room despite lighting changes.")
    
    # [Lab instructions for students would follow here]

