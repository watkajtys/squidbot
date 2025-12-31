import sys
import importlib
import os

def check_library(name):
    try:
        importlib.import_module(name)
        print(f"[OK] {name}")
        return True
    except ImportError:
        print(f"[FAIL] {name} not found. Run: pip install {name}")
        return False

def check_file(path):
    if os.path.exists(path):
        print(f"[OK] {path}")
        return True
    else:
        print(f"[FAIL] {path} missing.")
        return False

def main():
    print("--- SQUID DRONE SYSTEM CHECK ---\
")
    
    # 1. Check Libraries
    print("Checking Dependencies...")
    libs = [
        'numpy', 'cv2', 'RPi.GPIO', 'smbus2', 'flask',  # Core
        'gym_pybullet_drones', 'stable_baselines3', 'onnxruntime'  # AI & Sim
    ]
    results = [check_library(l) for l in libs]
    
    if all(results):
        print("\nAll libraries installed.")
    else:
        print("\nSome libraries are missing.")

    # 2. Check Directories
    print("\nChecking Project Structure...")
    paths = [
        'src/drivers', 
        'src/utils', 
        'docs/hardware_reference.md',
        'simulation/quadrotor.urdf',
        'tools/setup_pi.sh'
    ]
    results_path = [check_file(p) for p in paths]

    print("\n--- END CHECK ---")

if __name__ == "__main__":
    main()
