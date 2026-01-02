import time
# from drivers.msp import MSPDriver
# from drivers.tof_array import ToFArray
from utils.logger import AsyncLogger

def main():
    print("Squid Drone System Starting...")
    
    # 1. Initialize Hardware
    # msp = MSPDriver()
    # sensors = ToFArray()
    logger = AsyncLogger()
    
    # 2. The Game Loop
    Hz = 50
    dt = 1.0 / Hz
    
    try:
        while True:
            loop_start = time.perf_counter()
            
            # --- SENSE ---
            # dist = sensors.read_distances()
            
            # --- THINK ---
            # if dist < 200: throttle = 1000 (STOP)
            
            # --- ACT ---
            # msp.send_motor_command([throttle, throttle, throttle, throttle])
            
            # --- LOGGING (Lab 1.4) ---
            # Push data to the queue. This is non-blocking!
            # logger.log({
            #     "timestamp": loop_start,
            #     "roll": 0.0,
            #     "m1": 0.0
            # })

            # --- SLEEP (Lab 1.3) ---
            elapsed = time.perf_counter() - loop_start
            sleep_time = dt - elapsed
            
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                print(f"[WARNING] Lag Detected! Loop took {elapsed*1000:.2f}ms")
                
    except KeyboardInterrupt:
        print("Emergency Stop!")
        logger.close()
        # msp.disarm()

if __name__ == "__main__":
    main()
