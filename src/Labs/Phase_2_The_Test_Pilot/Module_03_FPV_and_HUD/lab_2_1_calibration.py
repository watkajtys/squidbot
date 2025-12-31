
SQUID DRONE CURRICULUM: lab_2_1_calibration.py
----------------------------------
Context: ../../../../curriculum/Phases/Phase_2_The_Test_Pilot/Module_03_FPV_and_HUD/Module_03_Lecture.md
Theory: curriculum/Library/
Win Condition: Phase II: Virtual Level (SQUID_GAMES.md)

This lab is part of the 'Soldering Iron to Swarm' masterclass.

import cv2
import numpy as np
import os
import glob

def calibrate_camera(image_dir, board_size=(9, 6), square_size=0.025):
    """
    Finds the Camera Intrinsic Matrix (K) and Distortion Coefficients.    
    Args:
        image_dir: Folder containing .jpg images of a chessboard.
        board_size: Number of internal corners (cols, rows).
        square_size: Real-world size of one square in meters.
    """
    # Termination criteria for sub-pixel accuracy
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # Prepare object points (0,0,0), (1,0,0), (2,0,0) ...
    objp = np.zeros((board_size[0] * board_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:board_size[0], 0:board_size[1]].T.reshape(-1, 2)
    objp *= square_size

    obj_points = [] # 3d point in real world space
    img_points = [] # 2d points in image plane. 

    images = glob.glob(os.path.join(image_dir, '*.jpg'))
    
    if not images:
        print(f"Error: No images found in {image_dir}")
        return

    print(f"Processing {len(images)} images...")

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, board_size, None)

        if ret:
            obj_points.append(objp)
            
            # Refine corners for higher accuracy
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            img_points.append(corners2)

            # Draw and display the corners (optional)
            cv2.drawChessboardCorners(img, board_size, corners2, ret)
            cv2.imshow('Calibration', img)
            cv2.waitKey(100)

    cv2.destroyAllWindows()

    # Calibrate
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, gray.shape[::-1], None, None)

    if ret:
        print("\n--- Calibration Successful ---")
        print("Intrinsic Matrix (K):")
        print(mtx)
        print("\nDistortion Coefficients (D):")
        print(dist)
        
        # Save results
        np.savez('camera_params.npz', mtx=mtx, dist=dist)
        print("\nParameters saved to 'camera_params.npz'")
    else:
        print("Calibration failed.")

if __name__ == "__main__":
    print("Squid Drone: Lab 3 - Camera Calibration")
    print("Ensure you have at least 10-20 photos of the calibration board in 'data/calibration_images/'")
    
    # Create directory if it doesn't exist
    os.makedirs('data/calibration_images', exist_ok=True)
    
    # In a real lab, students would capture these photos first.
    calibrate_camera('data/calibration_images')
