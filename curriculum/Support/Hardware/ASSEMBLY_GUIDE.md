[Return to Course Map](../../../../COURSE_MAP.md) | [Phase I Index](../../Phases/Phase_1_The_Mechanic/Module_00_The_Build/Module_00_Lecture.md)

---

# The Mechanic's Handbook: Assembly & Mechanics
**"If it vibrates, it will loosen. If it spins, it will cut."**

Wiring is only half the battle. The physical assembly of a drone determines if it survives the first crash.

---

## 1. The Capacitor "L-Bend" Technique
Capacitors are tall. Frames are flat. To fit the capacitor inside the Pavo20 frame:

1.  **Strain Relief:** Grip the capacitor legs with needle-nose pliers about **2mm** away from the body.
2.  **The Bend:** Bend the legs 90 degrees *against the pliers*. **DO NOT** bend against the capacitor body (you will break the seal).
3.  **Result:** The capacitor now lays flat (horizontal), parallel to the Flight Controller, instead of standing up like a tower.
4.  **Insulate:** Ensure the exposed legs are covered in heat shrink so they don't touch the carbon fiber frame (Carbon conducts electricity!).

---

## 2. Blue Loctite (The "Vibe Killer")
**Rule:** Every metal screw going into a metal hole needs **Blue Threadlocker (Loctite 243)**.
*   **Why:** Motors vibrate at 20,000 RPM. Without Loctite, screws back out. Motors fall off.
*   **Where:** Motor screws, Standoff screws, Camera screws.
*   **WARNING:** **Never** use Loctite on plastic (like the Pavo20 frame hoops or nylon nuts). The chemical reaction melts plastic, causing it to crack instantly.

---

## 3. Bind BEFORE You Build
**The Trap:** The "Bind Button" on your receiver (ELRS/TBS) is tiny.
**The Reality:** Once you mount the Raspberry Pi on top of the Flight Controller, that button is buried under a sandwich of silicon.
**The Fix:**
1.  Power up the Flight Controller via USB.
2.  Bind your Radio.
3.  Verify the connection in Betaflight (Receiver Tab).
4.  *Then* assemble the drone.

---

## 4. Propeller Orientation (The "High Five")
Props are not identical. There are **CW (Clockwise)** and **CCW (Counter-Clockwise)** props.
*   **The Scoop:** Look at the blade. The "High Edge" is the leading edge. It scoops the air.
*   **The Logic:**
    *   If the motor spins CW, use a CW prop.
    *   If you mismatch them, the drone pushes air *up* and sucks itself into the floor.
*   **"Props Out" Configuration:**
    *   Most Cinewhoops (like Squid) run "Props Out."
    *   Front Left & Rear Right: Spin **Counter-Clockwise**.
    *   Front Right & Rear Left: Spin **Clockwise**.
    *   *Why?* It prevents the front props from throwing grass/dirt onto the camera lens.

---

## 5. The USB Problem
**The Trap:** The Pavo20 frame hoop often blocks the USB-C port on the Flight Controller.
**The Fix:**
*   **Option A:** Use a Right-Angle USB-C adapter.
*   **Option B (Pro):** Use a **Magnetic USB Cable**. Leave the magnetic "Tip" plugged into the Flight Controller permanently. This protects the port from dirt and wear.

---

## 6. Soft Mounting (The "Gummies")
**The Component:** Those little rubber rings that came with your Flight Controller.
**The Rule:** USE THEM.
*   **Why:** The Gyroscope is sensitive. Hard-mounting the board transfers frame resonance (noise) to the sensor.
*   **Result:** The drone thinks it's shaking when it isn't, causing "Hot Motors" and poor battery life.
*   **Assembly:** Screw -> Frame -> **Rubber Gummy** -> FC Board -> Nut.

---

## 7. The Smoke Stopper (Final Warning)
Before you plug in the battery for the first time after assembly:
1.  Inspect all solder joints with a magnifying glass. Look for "Solder Balls" (tiny splashes of lead).
3.  Shake the drone upside down.
4.  Use the **Smoke Stopper**.

---

## 8. The Battery Strap Trap
**The Problem:** The Pavo20 frame is designed for a naked stack. By adding the Raspberry Pi and spacers, you have added ~15mm of height. The stock battery strap might be too short.
**The Fix:**
*   **Link Up:** If the strap is too short, you can link two small zip-ties together to create a temporary "harness."
*   **Velcro:** Use a piece of adhesive Velcro between the battery and the mounting plate to prevent it from sliding out sideways.
*   **Do Not Crush:** Ensure the strap is tight enough to hold the battery, but not so tight that it bows the Raspberry Pi PCB.

---

## 9. Sensor Placement Strategy (The "Squid" Layout)
Space is tight. Don't crowd the stack.

### 9.1 The "Side-Pod" (Upward Lidar)
**Don't** mount the Upward Lidar on top of the stack. It blocks the battery.
*   **The Fix:** Mount it to the top of the **Plastic Propeller Duct** (Left or Right side).
*   **Method:** A small zip-tie or double-sided tape holding the sensor to the duct hoop.
*   **Direction:** Ensure it points straight UP ($Z+$ axis).

### 9.2 The "Belly Button" (Downward Lidar & Flow)
**Location:** The underside of the frame.
*   **The Fix:** Find a gap in the carbon fiber baseplate.
*   **Insulate:** Put electrical tape between the sensor PCB and the carbon frame.
*   **Secure:** Zip-ties or Hot Glue.
*   **Direction:** Points straight DOWN ($Z-$ axis).

### 9.3 The "Cyclops" (Camera)
**Location:** Front Nose.
*   **The Fix:** Double-sided foam tape + Zip tie to the front plastic brace.
*   **Cable:** Route the ribbon cable *under* the Flight Controller if possible to keep it away from props.

### 9.4 The "Unicorn" (Front Lidar)
**Location:** Forehead (Above the Camera).
*   **The Fix:** Mount the VL53L5CX on top of the plastic camera hoop.
*   **The Tilt:** Tilt it **UP** by ~10 degrees.
*   **Why:** When the drone flies forward, it pitches down. If the sensor is flat, it looks at the floor. The tilt keeps it looking at the wall.

### 9.5 The "Tail" (GPS & Compass)
**Location:** The absolute rear.
*   **The Enemy:** Magnetic Interference from power wires (EMI).
*   **The Fix:** Mount it as far back as possible, behind the battery connector.
*   **The "Zip-Tie Stinger" Hack:** 
    *   Take a large, thick zip-tie and strap it to the rear of the frame so it sticks out like a tail.
    *   Tape or zip-tie the GPS module to the end of this "boom."
    *   **Benefit:** This moves the compass away from the electrical noise and survives crashes by bending rather than breaking.
*   **Orientation:** The Ceramic Square must face the **SKY**. The Arrow on the board must point **FORWARD**.

---

## 10. The Secret Menu (Pro Hacks)

### 10.1 The "Inward Solder" (Pad Saver)
**The Trap:** Soldering battery wires pointing *out*. In a crash, the battery ejects and rips the pads off the board.
**The Fix:** Solder the wires pointing **INWARD** (towards the center of the board), then loop them back out in a U-turn around a standoff.
**The Physics:** When the battery pulls, the wire pulls against the **Standoff**, not the solder joint. The standoff is steel; the pad is foil. Steel wins.

### 10.2 The "Sensor Foam" (Barometer Shield)
**The Trap:** Altitude hold is erratic outdoors or near fans.
**The Cause:** The Barometer is sensitive to wind gusts and light.
**The Fix:** Glue a small chunk of **Open Cell Foam** (sponge) over the barometer chip. It allows air pressure in but blocks the wind "noise" from the props.





