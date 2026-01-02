# Module 0: Foundational Engineering for Autonomous Aerial Robotics

## 1. Introduction: The Full-Stack Challenge of Autonomous Flight

The design, implementation, and operation of an autonomous quadrotor represent one of the most demanding interdisciplinary challenges in modern engineering. Unlike a ground vehicle, which operates in a state of static stability—simply halting if a processor hangs or a sensor drifts—an aerial robot exists in a continuous state of dynamic instability. It requires a relentless, high-frequency control loop merely to maintain its position in three-dimensional space. A failure in any subsystem, whether it is a transient voltage dip in the power distribution network, a nanosecond-scale jitter in the operating system scheduler, or a mathematical singularity in the orientation logic, results not merely in an error log or a graceful degradation of service, but in the catastrophic physical destruction of the machine.

Module 0 serves as the foundational bedrock for this course. Before writing a single line of control code, soldering a wire, or training a neural network, the roboticist must understand the first principles governing the machine. We are not merely assembling components; we are architecting a complex system where physics, mathematics, and computation interact in real-time. This report provides an exhaustive analysis of the critical subsystems that constitute the autonomous drone: the electro-chemical realities of power delivery, the stochastic nature of real-time computing on the Raspberry Pi Zero 2 W, the rotational mechanics of quaternions versus Euler angles, and the aerodynamic principles of differential thrust and signal processing.

This analysis prioritizes "deep intuition" and "mechanical sympathy" over rote calculation. While the flight controller software will handle the matrix multiplications and the integration of differential equations, the engineer must understand why those matrices are constructed, how the hardware limits the mathematical precision, and where the physical implementation typically fails. We will explore the "why" behind the math and the physics, bridging the gap between abstract theory and the harsh reality of flying robots.

---

## 2. The Physics of Power: Energy Storage and Delivery

The lifeblood of any autonomous robot is its power system. In the context of a quadrotor, the power system is subjected to extreme transient loads that stress the electrochemical limits of batteries and the regulation capabilities of power conversion circuits. The dynamics of energy storage and delivery are often the primary source of system instability, manifesting as "brownouts" or erratic behavior that software cannot correct.

### 2.1 Lithium Polymer (LiPo) Electrochemistry and Internal Resistance

We utilize Lithium Polymer (LiPo) batteries not arbitrarily, but due to their high specific energy and discharge rates, which are essential for overcoming the relentless pull of gravity. However, treating a battery as an ideal voltage source—an infinite pool of electrons at a constant potential—is a catastrophic error in robotics. A LiPo battery must be modeled as an ideal voltage source ($V_{ideal}$) in series with a variable resistor ($R_{internal}$).

#### 2.1.1 The Solid Electrolyte Interphase (SEI) Layer

The internal resistance (IR) of a LiPo cell is not a static value found in a datasheet; it is a dynamic, living property determined by the cell's chemistry and historical usage. A critical factor governing IR is the Solid Electrolyte Interphase (SEI) layer. The SEI is a passivation layer that forms on the anode surfaces from the decomposition of the electrolyte during the initial charging cycles. [1]

While the SEI is necessary to prevent further electrolyte decomposition and stabilize the cell, it acts as a resistive barrier to lithium ion transport. As a battery ages or is subjected to high thermal loads, this layer thickens. The thicker the SEI layer, the higher the internal resistance becomes. [1] This degradation is significantly accelerated by keeping the battery at high voltage (fully charged) for extended periods or exposing it to high temperatures, which promotes the oxidation of the electrolyte and further deposition on the anode. [3]

The formation of the SEI layer is also influenced by the mechanical stresses of charging and discharging. High currents can cause micro-cracking in the electrode materials, exposing fresh surface area where new SEI layers form, consuming lithium ions and electrolyte in the process. This "lithium inventory loss" leads to capacity fade, while the thickening SEI layer leads to power fade (increased resistance). [4] The battery effectively becomes "stiffer" electrically; it holds energy, but it cannot release it quickly.

#### 2.1.2 Voltage Sag and Ohm’s Law: The Mechanism of Failure

The practical manifestation of internal resistance is "voltage sag." When the flight controller demands a sudden increase in motor thrust (a maneuver known as a "punch-out"), the current draw ($I_{load}$) spikes effectively instantly. According to Ohm’s Law, a voltage drop occurs across the internal resistance of the battery:

$$V_{drop} = I_{load} \times R_{internal}$$

The voltage available to the electronics and motors at the battery terminals ($V_{terminal}$) is the nominal chemical voltage minus this drop:

$$V_{terminal} = V_{nominal} - (I_{load} \times R_{internal})$$

This relationship reveals why a battery can show a healthy "resting" voltage but fail immediately under load. If a battery has developed a high $R_{internal}$ (due to a thick SEI layer, poor quality, or age), a 20A current spike can cause the voltage to sag below the critical brownout threshold of the flight computer or Electronic Speed Controllers (ESCs). [6]

This phenomenon effectively reduces the "usable" capacity of the battery at high discharge rates. A battery might deliver 1000mAh at a gentle 1A draw, but only 500mAh at a violent 20A draw before the voltage sags below the safe cutoff. [8] This is often referred to as the Peukert Effect (though technically originally for lead-acid, the principle of efficiency loss at high load applies here). The "C-rating" of a battery is theoretically the maximum safe discharge rate, but in reality, it is a thermal limit defined by how much heat ($P = I^2R$) the internal resistance generates before the cell destroys itself. [1]

### 2.2 Voltage Regulation: The Battery Eliminator Circuit (BEC)

The motors require raw battery voltage (e.g., 7.4V - 14.8V) to generate thrust, but the Raspberry Pi, flight controller, and sensors require a stable, precise 5V logic level. The component responsible for this step-down conversion is the Battery Eliminator Circuit (BEC). Understanding the topology of the BEC is crucial for system stability and sensor integrity, as the wrong choice can lead to overheating or signal corruption.

#### 2.2.1 Linear vs. Switching Regulators: A Critical Trade-off

There are two primary topologies for regulating voltage in robotics: Linear Regulators and Switching Regulators (often called SBECs or UBECs).

| Parameter | Linear BEC | Switching BEC (SBEC/UBEC) |
| :--- | :--- | :--- |
| **Operational Principle** | Acts as a variable resistor, burning excess voltage as heat. | Uses high-frequency switching (PWM) with an inductor/capacitor to store and transform energy. |
| **Efficiency** | Low (40-70%). Efficiency drops linearly as the input voltage rises relative to output. | High (80-95%+). Efficiency remains relatively constant across input voltages. |
| **Heat Generation** | High. $P_{heat} = (V_{in} - V_{out}) \times I_{load}$. | Minimal. Heat is largely independent of the voltage differential. |
| **Noise Characteristics** | Very Low. Output is smooth and clean (DC). | High. Generates Electromagnetic Interference (EMI) due to switching frequency. |
| **Transient Response** | Very Fast. Reacts almost instantly to load changes. | Slower. Requires feedback loop cycles to adjust to load spikes; potential for ripple. |
| **Primary Application** | Sensitive analog sensors, audio circuits, low-power microcontrollers. | High-power avionics, flight computers, servos, telemetry radios. |

For a quadrotor, a Switching BEC is almost universally required due to the current demands of the modern flight computer and peripherals. A linear regulator stepping a 12V (3S LiPo) source down to 5V for a 2A load (Pi + peripherals) would need to dissipate $(12V - 5V) \times 2A = 14 \text{ Watts}$ of heat. In a small enclosed drone chassis, 14 Watts is a massive thermal load that would likely trigger thermal shutdown or melt components. [9]

However, the "switching" action of an SBEC creates a new problem: Electromagnetic Interference (EMI). The regulator switches the input voltage on and off thousands of times per second (typically 300kHz to 1.5MHz). This creates a high-frequency noise signature that can couple into adjacent wiring, particularly affecting sensitive radio receivers (GPS, RC link) or analog sensor lines (magnetometers). [9] Therefore, while we select SBECs for their thermal efficiency, we must be vigilant about wire routing, keeping power lines twisted and away from antennas, and potentially using ferrite rings to filter high-frequency noise from the output lines. [9]

### 2.3 The Brownout Danger Zone: Raspberry Pi Power Architecture

The Raspberry Pi Zero 2 W is a powerful computer, but it is electrically fragile compared to a microcontroller. It lacks the robust power filtering found on desktop motherboards and relies on a specific Power Management IC (PMIC) to manage its rails.

#### 2.3.1 The PMIC and Undervoltage Thresholds

The Raspberry Pi Zero 2 W uses a custom System-in-Package (SiP) that includes power management. The critical voltage threshold for the Pi is 4.63V ($\pm 5\%$). [11] The PMIC monitors the 5V rail voltage. If the input voltage drops below this level, even for a microsecond, the PMIC asserts an internal undervoltage flag.

In a desktop environment, a stable power supply makes this a non-issue. In a drone application, we operate in a hostile electrical environment. The motors are large inductive loads. When a motor spins, it builds a massive magnetic field. If the current is suddenly throttled down (e.g., during a rapid deceleration or stabilization maneuver), that collapsing magnetic field generates a high-voltage spike, known as an inductive kickback or flyback voltage. [14]

While the ESCs typically handle this energy, ground loops, long wires, or poor impedance matching can allow these transients to affect the main 5V rail. Furthermore, if the BEC cannot react fast enough to a simultaneous load transient—for example, the Pi's four cores ramping to 100% utilization at the exact moment the Wi-Fi radio transmits—the voltage may dip momentarily.

If the voltage hits 4.63V, the Pi generally takes defensive action. It may throttle the CPU frequency to reduce power consumption, which induces unexpected latency in the control loop. [11] If the voltage drops further, a brownout occurs—the logic state of the processor becomes indeterminate, leading to a system reset (crash). [17]

**The Key Insight:** The "Brownout" is rarely a failure of the battery's total capacity (Energy). It is almost always a failure of impedance (Power delivery). It is caused by high internal resistance in the battery creating sag, or resistance in the cables and connectors preventing the BEC from delivering adequate voltage during a transient spike. This is why high-quality connectors (XT30/XT60) and low-ESR capacitors are mandatory in drone builds, not optional upgrades.

---

## 3. Computational Hardware: The Brain (Raspberry Pi Zero 2 W)

Aligning with Stanford labs, this module utilizes the Raspberry Pi Zero 2 W. This board offers a massive leap in performance over traditional microcontrollers (like the Arduino or STM32 typically found on basic drones), enabling computer vision and advanced SLAM (Simultaneous Localization and Mapping). However, it introduces the immense complexity of running a non-deterministic General Purpose Operating System (Linux) on a complex processor architecture.

### 3.1 The Processor: ARM Cortex-A53 Architecture

The heart of the Zero 2 W is the Broadcom BCM2710A1, a System-in-Package (SiP) containing a quad-core 64-bit ARM Cortex-A53 processor. [18] To code effective low-level controls, one must understand the silicon that executes them.

The Cortex-A53 is an efficient, in-order execution pipeline processor. Unlike high-performance desktop CPUs (which use out-of-order execution to optimize instruction flow), the in-order nature of the A53 means that if one instruction stalls (waiting for data), the entire pipeline behind it stalls. [21] This makes the processor highly sensitive to memory latency.

Furthermore, the four cores share a Level 2 (L2) cache. This shared resource becomes a point of contention. If Core 0 (running the OS background tasks) flushes the L2 cache, Core 1 (running the flight control loop) may experience a sudden "cache miss" when it tries to access its control variables. It must then fetch data from the relatively slow main RAM (DDR2), adding hundreds of CPU cycles of latency. [22] This hardware-level resource contention is a primary source of "jitter" in the control loop—unpredictable variations in execution time that can destabilize the flight.

### 3.2 The "Double Interrupt" and Latency Challenges

A specific and subtle phenomenon noted in technical literature regarding the Cortex-A53 and Linux integration is the "Double Interrupt" or interrupt latency issue. [23]

In a standard Linux environment, when a hardware interrupt occurs (e.g., the gyroscope signaling that new data is ready via a GPIO pin), the kernel must suspend the current process, switch contexts, and execute the interrupt handler. On the A53, complex interactions between the Generic Interrupt Controller (GIC v4) and the Linux kernel can sometimes lead to excessive latency or "double" handling scenarios. [25]

This manifests when the interrupt controller signals the CPU, but due to kernel locking or priority handling, the CPU delays the context switch. In some reported cases on A53 SoCs, the interrupt logic effectively "fires twice" or requires a double context switch to properly resolve the handler, significantly magnifying the overhead. [27] Measurements have shown that without specific real-time optimizations, interrupt latencies on A53-based Linux systems can spike to tens or even hundreds of microseconds. [29]

For a web server, a 100$\mu$s delay is irrelevant. For a flight controller running a 400Hz loop (2.5ms period), a 100$\mu$s delay represents a 4% phase lag. In control theory, **phase lag** reduces the phase margin of the system. If the drone reacts to a gust of wind 100$\mu$s later than the math assumes, the correction may arrive when the drone has already rotated past the setpoint, leading to oscillation or instability.

### 3.3 Linux as a Control OS: Throughput vs. Determinism

Standard Linux is a General Purpose Operating System (GPOS). Its scheduler, the Completely Fair Scheduler (CFS), is mathematically designed to maximize throughput—the total amount of work done over time—and to ensure fairness among all running processes. It is not designed for determinism—the guarantee that a specific task will finish by a specific deadline.

In robotics, we care almost exclusively about the worst-case execution time (WCET), not the average. If the scheduler decides to prioritize a background system update or a Wi-Fi handshake over the flight control loop for 10 milliseconds, the drone acts blindly for that duration, potentially flipping over.

#### 3.3.1 The PREEMPT_RT Patch: Bridging the Gap

To mitigate this fundamental mismatch, roboticists often utilize the PREEMPT_RT kernel patch. [30] This patch transforms Linux into a Real-Time Operating System (RTOS) through several invasive changes to the kernel's behavior:

*   **Fully Preemptible Kernel:** In standard Linux, if the kernel is executing a system call (like writing to a file) on behalf of a low-priority process, it generally cannot be interrupted, even by a high-priority flight control task. PREEMPT_RT allows high-priority tasks to interrupt the kernel almost anywhere, drastically reducing the "blocking time". [30]
*   **Sleeping Spinlocks:** Standard kernels use "spinlocks" where a thread waiting for a resource sits in a tight loop burning CPU cycles. PREEMPT_RT replaces these with mutexes that allow the waiting thread to sleep, freeing the CPU for other tasks, while implementing priority inheritance to prevent deadlocks. [30]
*   **High-Resolution Timers:** It optimizes the timer interrupt system to allow for much finer-grained scheduling events, essential for high-frequency control loops.

Even with PREEMPT_RT, the Raspberry Pi Zero 2 W remains a "Soft Real-Time" system. We can guarantee deadlines most of the time, with high probability, but not with the absolute mathematical certainty of a microcontroller-based "Hard Real-Time" system. [30] Therefore, our software design must be robust enough to handle the occasional missed deadline (jitter) without catastrophic failure. We design for "mechanical sympathy"—understanding that the computer will occasionally blink, and ensuring the drone can glide through that blink.

---

## 4. Actuation and Aerodynamics: From Code to Motion

The transition from digital commands to physical forces occurs at the Electronic Speed Controllers (ESCs) and motors. This is where the abstract control signals meet the resistance of the air.

### 4.1 Brushless DC Motor Physics

We use Brushless DC (BLDC) motors. Unlike brushed motors, which use mechanical commutators to switch current direction (and thus magnetic fields), BLDC motors rely on the ESC to switch the phases electronically. This allows for higher efficiency and longevity but requires complex timing logic.

#### 4.1.1 The KV Rating and Back EMF

A fundamental specification of any BLDC motor is its KV rating (Velocity Constant). KV is defined as the RPM per volt applied, under a theoretical no-load condition. [34]

$$\text{RPM}_{no\_load} = KV \times V_{applied}$$

However, KV is not just a speed limit; it is deeply related to the motor's Back Electromotive Force (Back EMF). As the motor spins, the permanent magnets passing the stator coils induce a voltage in the coils that opposes the drive voltage (Lenz's Law). The faster the motor spins, the higher this Back EMF ($\epsilon$). The motor reaches its maximum speed when the Back EMF plus the resistive losses equals the supply voltage. [36]

The current flowing through the motor is governed by the difference between the supply voltage and this Back EMF:

$$I_{motor} = \frac{V_{supply} - \epsilon}{R_{winding}}$$

This equation reveals a critical insight: Torque is proportional to Current ($I$), and Current is driven by the headroom between Voltage and Back EMF.

At low RPM (start-up), Back EMF is near zero. The current is limited only by the very low winding resistance ($R_{winding}$), causing massive current spikes during rapid acceleration.

If the battery voltage sags (due to high internal resistance), the $V_{supply}$ term drops. As $V_{supply}$ gets closer to $\epsilon$, the current $I_{motor}$ drops, and torque vanishes. This is why a drone feels "sluggish" on a low battery—it literally cannot generate the electrical pressure difference required to push current into the spinning motor to change its speed. [37]

#### 4.1.2 Torque Generation ($K_t$)

For BLDC motors, torque ($\tau$) is directly proportional to current ($I$) via the torque constant ($K_t$):

$$\tau = K_t \times I$$

There is a fundamental physical relationship between $K_t$ and $K_v$: they are inversely proportional.

$$K_t \propto \frac{1}{K_v}$$

High KV motors have lower torque per amp. This dictates our design choice:

*   **Large propellers** have high rotational inertia and drag. They require high torque to spin up and change speed. Therefore, we use Low KV motors (e.g., 900KV).
*   **Small racing propellers** have low inertia but need high RPM to generate thrust. Therefore, we use High KV motors (e.g., 2300KV). Mismatching these (e.g., large prop on high KV motor) forces the motor to draw excessive current to generate torque, likely burning out the windings or the ESC. [34]

### 4.2 Quadrotor Dynamics: 6DOF from 4 Actuators

A quadrotor is an underactuated system. It operates in 6 Degrees of Freedom (X, Y, Z position; Roll, Pitch, Yaw orientation) but possesses only 4 control inputs (the speeds of the four motors). We achieve full control by "mixing" these inputs to couple the degrees of freedom.

*   **Throttle (Vertical Acceleration):** All four motors increase speed equally. Total thrust exceeds Weight.
*   **Pitch (Forward/Backward):** Differential thrust is applied between the front and rear motors. Increasing the rear motor speed while decreasing the front motor speed creates a torque around the Y-axis. The total thrust remains constant (to maintain altitude), but the imbalance rotates the drone nose-down. This tilts the thrust vector forward, creating a horizontal component that accelerates the drone. [39]
*   **Roll (Left/Right):** Similar to pitch, differential thrust is applied between the left and right motor pairs.

### 4.3 The Physics of Yaw: Newton’s Third Law

Yaw (rotation around the vertical Z-axis) is the most subtle and often the weakest control axis. It relies entirely on Newton’s Third Law: "For every action, there is an equal and opposite reaction."

When a motor applies torque to spin a propeller clockwise (CW), the aerodynamic drag on the propeller applies an equal and opposite reaction torque to the motor housing (and thus the drone frame) in the counter-clockwise (CCW) direction. [39]

In a standard quadrotor configuration:

*   Two motors (e.g., Front-Left, Rear-Right) spin Clockwise (CW). They push the frame CCW.
*   Two motors (e.g., Front-Right, Rear-Left) spin Counter-Clockwise (CCW). They push the frame CW.

In a stable hover, these reaction torques cancel out perfectly, and the drone holds its heading. To Yaw the drone to the right (CW):

1.  We increase the speed of the CCW spinning motors. This increases their CW reaction torque on the frame.
2.  We decrease the speed of the CW spinning motors. This decreases their CCW reaction torque.

The net result is a net Clockwise torque on the frame, causing the drone to yaw right. [39]

**Critical Insight:** Yaw authority is inherently weaker than Pitch/Roll authority. Pitch and Roll are generated by a "lever arm" effect—thrust forces acting at the end of the arms ($Force \times Distance$). Yaw is generated by aerodynamic drag torque, which is much smaller magnitude than the thrust force. This is why drones struggle to hold their heading in strong winds compared to holding their level, and why "yaw spin" maneuvers are slower than flips.

---

## 5. Signal Processing: Vibration and Filtering

The sensors (gyroscopes and accelerometers) on the drone are microscopic mechanical systems (MEMS). They are extremely sensitive to the violent mechanical environment of a quadrotor frame.

### 5.1 The Nature of Noise: The Frequency Domain

A quadrotor frame is a cacophony of vibrations. The motors, spinning at perhaps 15,000 RPM, generate a fundamental vibration frequency of 250Hz ($15,000 / 60$). If the propellers are unbalanced, or if a bearing is worn, this vibration transfers directly to the IMU (Inertial Measurement Unit).

To the flight controller, this vibration looks like "movement." If the gyro reports that the drone is oscillating at 250Hz, the controller might try to fight it.

### 5.2 FFT and Spectral Analysis

To understand this noise, we use the Fast Fourier Transform (FFT). The FFT is a mathematical algorithm that decomposes the time-domain signal (voltage vs. time) from the gyro into a frequency-domain graph (amplitude vs. frequency). [44]

*   **Motor Noise:** Appears as sharp spikes at the motor rotation frequency and its harmonics (2x, 3x). This frequency moves up and down as the throttle changes.
*   **Broadband Noise:** Wind, airflow turbulence, and frame resonance appear as "fuzz" or broad humps across the spectrum. [46]

Modern flight controllers run "Dynamic Notch Filters" that use FFT analysis in real-time to identify the motor noise spike and surgically remove it from the sensor data before it reaches the PID loop. [46]

### 5.3 The Nyquist-Shannon Sampling Theorem and Aliasing

This is arguably the most critical concept for digital control systems. The Nyquist-Shannon Sampling Theorem states that to accurately capture a continuous signal of frequency $f$, you must sample it at a rate of at least $2f$. [47]

$$f_{sample} > 2 \times f_{signal}$$

If we sample our gyroscope at 1kHz ($f_{sample} = 1000\text{Hz}$), the Nyquist Frequency is 500Hz. We can accurately see any movement up to 500Hz.

**The Trap: Aliasing.**

If there is a physical vibration at 600Hz (e.g., a bent prop at high RPM), the sensor will not just "miss" it. The signal will alias. It will "fold back" into the measurable spectrum, appearing as a lower frequency ghost signal.

$$f_{alias} = | f_{signal} - f_{sample} | = | 600 - 1000 | = 400\text{Hz}$$

The flight controller will see a 400Hz oscillation that does not exist in reality. It is a mathematical phantom caused by the sampling process. The PID loop may try to correct for this ghost motion, causing the motors to oscillate, which creates more vibration, leading to a feedback loop that can cause a "fly-away" or instant motor burnout. [49]

**Mitigation:**

*   **Mechanical Damping:** We soft-mount the flight controller on rubber gummies to physically absorb high frequencies before they reach the sensor.
*   **Analog Low-Pass Filtering:** Most IMUs have built-in hardware filters to cut frequencies above the Nyquist limit before the digitization step.
*   **Software Filtering:** However, every filter introduces phase delay (latency). A filter that smooths the data effectively reports "what happened a few milliseconds ago." Too much filtering stabilizes the data but destabilizes the drone due to lag. [49]

---

## 6. Mathematics of Orientation: The "Why"

To control the drone, we need to mathematically represent its orientation (attitude) in 3D space. There are three common methods, each with a progression of complexity and capability.

### 6.1 Euler Angles: Intuitive but Flawed

Euler angles represent orientation as three sequential rotations: Roll ($\phi$), Pitch ($\theta$), and Yaw ($\psi$). This is intuitive for humans—we can easily visualize "pitching up 20 degrees."

**The Problem: Gimbal Lock.**

Euler angles suffer from a mathematical singularity known as Gimbal Lock. This occurs when two of the three rotational axes align, causing a loss of one degree of freedom. [51]

Imagine the drone pitches up 90 degrees (nose pointing straight up). In this orientation, "Yaw" (spinning around the Z-axis) and "Roll" (spinning around the X-axis) become the exact same physical motion relative to the horizon.

The math breaks down. The system can no longer distinguish between roll and yaw changes. The transformation matrices become singular (the determinant is zero). In a simulation or a flight controller, this causes the solution to oscillate or flip wildly. The controller may command a violent correction to "fix" the ambiguity, crashing the vehicle. [51]

### 6.2 Complex Numbers: The 2D Rotation Operator

To solve this, we must look at rotation differently. Consider the Complex Plane. A complex number $z = x + iy$ can be viewed as a vector in 2D space.

Multiplying a number by $i$ rotates it 90 degrees counter-clockwise. Multiplying by $i^2$ (or $-1$) rotates it 180 degrees. Multiplying by a complex number of unit length $e^{i\theta} = \cos\theta + i\sin\theta$ rotates a vector by angle $\theta$. [55]

This works perfectly in 2D. It is stable, smooth, and has no singularities. We intuitively want to extend this "multiplication = rotation" concept to 3D.

### 6.3 Quaternions: The 4D Rotation Operator

Sir William Rowan Hamilton spent years trying to find a 3D analog (triplets). He eventually discovered that you cannot simply add one more imaginary unit. You need three imaginary units ($i, j, k$) and a real scalar ($w$), creating a 4-dimensional system. This gives us the Quaternion:

$$q = w + xi + yj + zk$$

Where $w$ is the scalar (real) part, and $x, y, z$ form the vector part. The fundamental rules are $i^2 = j^2 = k^2 = ijk = -1$. [56]

**Why Quaternions Work for Robotics:**

*   **No Gimbal Lock:** Quaternions operate on the surface of a 4D hypersphere ($S^3$). This shape has no poles or singularities where the math breaks down. You can represent any 3D orientation smoothly and continuously.
*   **Computational Efficiency:** Composing two rotations using Euler angles requires multiplying 3x3 matrices (27 floating-point multiplications). Multiplying two quaternions requires only 16 multiplications. [56] On a constrained processor like the Pi Zero, this efficiency matters.
*   **SLERP (Spherical Linear Interpolation):** If you want to smoothly transition the drone from Orientation A to Orientation B, Euler angles are messy (the path might wobble). Quaternions allow for SLERP, which finds the shortest path ("great circle") across the 4D hypersphere, resulting in the smoothest possible rotation in 3D space. [51]

**Intuition:** Think of a quaternion not as a set of angles, but as an Axis-Angle representation encoded into 4 numbers. It essentially says "Rotate $\theta$ degrees around this specific 3D vector $\vec{v}$."

$$q = (\cos(\theta/2), \vec{v}\sin(\theta/2))$$

**The Sandwich Product:**

To rotate a 3D vector $\vec{v}$ using a quaternion $q$, we use the "sandwich" product:

$$\vec{v}' = q \vec{v} q^{-1}$$

We multiply the vector by the quaternion on one side and its inverse (conjugate) on the other. This double-sided multiplication is necessary to constrain the result to 3D space (pure vector) rather than rotating it out into the 4th dimension. [56]

---

## 7. Communication and Telemetry

The drone must communicate its state to the ground station and receive commands. The choice of network protocol is a trade-off between reliability and timeliness.

### 7.1 TCP vs. UDP in Robotics

**TCP (Transmission Control Protocol):** TCP is connection-oriented. It guarantees delivery and order. If packet #50 is lost, the receiver asks the sender to retransmit it, and pauses the delivery of packets #51-100 until #50 arrives.

The Problem: This is known as Head-of-Line (HOL) Blocking. [58] In a real-time control link or video feed, we do not care about what happened 500ms ago. We need the now. Pausing the stream to recover an old packet causes lag ("stutter"), which is fatal for teleoperation.

**UDP (User Datagram Protocol):** UDP is connectionless. It sends packets as fast as possible ("fire and forget"). If a packet drops, it is gone forever.

Why we use it: It has no HOL blocking. If a video frame is corrupted, the screen glitches for a millisecond, but the stream continues instantly with the next frame. For robotics telemetry and control, low latency is more valuable than perfect reliability. [60]

**Emerging Tech: QUIC.**

Newer protocols like QUIC (used in HTTP/3) attempt to bridge this gap, offering the reliability of TCP with the non-blocking multiplexing of UDP, and are becoming common in modern drone links. [58]

---

## 8. Regulatory Context: Operating Legally

Finally, the engineering must exist within the legal framework. The FAA regulates the National Airspace System (NAS), and these laws apply to university projects as much as commercial airlines.

*   **Part 107 vs. Recreational:** Unless you are flying strictly for personal enjoyment, you generally fall under Part 107 (commercial rules). University research often blurs these lines, but recreational exemptions typically apply to students learning the basics. [62]
*   **Visual Line of Sight (VLOS):** You must be able to see the drone with unaided eyes (no binoculars, no FPV goggles as primary view) at all times. [63] This physically limits your operational range, regardless of your radio link range.
*   **Remote ID (2025):** New regulations require drones >250g (and all registered drones) to broadcast a digital "license plate" signal (location, ID) via Wi-Fi/Bluetooth. [65] Since our Raspberry Pi Zero 2 W setup likely exceeds 250g, the system architecture must account for Remote ID compliance, either via a hardware module or a software stack if permitted.

---

## 9. Conclusion

Module 0 is not just a collection of parts and equations; it is a lesson in constraints.

*   The **Battery** constrains our power through Internal Resistance and Voltage Sag.
*   The **BEC** constrains our electrical noise floor via Switching EMI.
*   The **Computer (A53)** constrains our real-time performance via Jitter and Interrupt Latency.
*   The **Sensors** constrains our knowledge of the world via Nyquist Aliasing.
*   The **Math (Euler/Quaternion)** constrains how we can move without singularities.

Understanding these constraints allows us to design a system that flies with the physics, rather than fighting against them. We do not just solve for $X$; we engineer the system that allows $X$ to exist in the real world.

---

## Works Cited

1. accessed January 1, 2026, http://www.theampeer.org/lipo-intro/lipo-intro.html#:~:text=Unfortunately%20the%20SEI%20layer%20results,because%20of%20decreased%20lithium%20transportability.
2. Internal Resistance of a Battery | Ossila, accessed January 1, 2026, https://www.ossila.com/pages/internal-resistance-of-a-battery
3. Lithium Polymer Battery Technology - An Introduction, accessed January 1, 2026, http://www.theampeer.org/lipo-intro/lipo-intro.html
4. Correlating long-term lithium ion battery performance with solid electrolyte interphase (SEI) layer properties - TRACE: Tennessee Research and Creative Exchange, accessed January 1, 2026, https://trace.tennessee.edu/cgi/viewcontent.cgi?params=/context/utk_graddiss/article/6087/&path_info=Dissertation_Seong_Jin_AN_SEI_v15.pdf
5. The Science Behind Lithium Battery Capacity Loss, accessed January 1, 2026, https://www.large-battery.com/blog/what-causes-capacity-loss-of-lithium-battery-explained/
6. LiPo Battery Internal Resistance Testing - Flite Test, accessed January 1, 2026, https://www.flitetest.com/articles/LiPo_Battery_Internal_Resistance_Testing
7. Ep. 165: What is Voltage Sag? LiPo Battery Voltage Sag & Internal Resistance Explained, accessed January 1, 2026, https://www.youtube.com/watch?v=0KLOXEWrOes
8. LiPo Battery Voltage, Discharge Rate and Cycle Life | Grepow, accessed January 1, 2026, https://www.grepow.com/blog/basis-of-lipo-battery-specifications.html
9. Battery Eliminator Circuit Design Guide | BECs - GNS, accessed January 1, 2026, https://gnsems.com/what-is-a-battery-eliminator-circuit-bec-and-how-does-it-work/
10. battery eliminator circuit drone - PCB & MCPCB - Best Technology, accessed January 1, 2026, https://www.bestpcbs.com/blog/tag/battery-eliminator-circuit-drone/
11. Raspberry Pi 3+ "brownouts" and under voltage issue with good power supplies, accessed January 1, 2026, https://raspberrypi.stackexchange.com/questions/143143/raspberry-pi-3-brownouts-and-under-voltage-issue-with-good-power-supplies
12. Official power supply browns out with RPi 4B, voltage specifications mismatched, accessed January 1, 2026, https://forums.raspberrypi.com/viewtopic.php?t=290853
13. Raspberry voltage requirement, accessed January 1, 2026, https://forums.raspberrypi.com/viewtopic.php?t=324813
14. Inductive Kickback – The Mower Project, accessed January 1, 2026, https://mowerproject.com/2021/01/07/inductive-kickback/
15. Inductive kickback made simple to grasp, easy to handle, accessed January 1, 2026, https://inductive-kickback.com/2019/04/inductive-kickback-made-simple-to-grasp-easy-to-handle/
16. How Raspbian Detects Under Voltage - Raspberry Pi Stack Exchange, accessed January 1, 2026, https://raspberrypi.stackexchange.com/questions/60593/how-raspbian-detects-under-voltage
17. Pi3 reboot; motor noise, brownout or? - Raspberry Pi Forums, accessed January 1, 2026, https://forums.raspberrypi.com/viewtopic.php?t=231474
18. SC1176 Raspberry Pi | Embedded Computers - DigiKey, accessed January 1, 2026, https://www.digikey.com/en/products/detail/raspberry-pi/SC1176/15298147
19. Buy a Raspberry Pi Zero 2 W, accessed January 1, 2026, https://www.raspberrypi.com/products/raspberry-pi-zero-2-w/
20. Raspberry Pi Zero 2 W, accessed January 1, 2026, https://datasheets.raspberrypi.com/rpizero2/raspberry-pi-zero-2-w-product-brief.pdf
21. Arm Cortex-A53 MPCore Processor Technical Reference Manual - NXP Community, accessed January 1, 2026, https://community.nxp.com/pwmxy87654/attachments/pwmxy87654/Layerscape/8583/1/DDI0500J_cortex_a53_trm.pdf
22. A53 latencies / jitter (memory access)? - Architectures and ..., accessed January 1, 2026, https://community.arm.com/support-forums/f/architectures-and-processors-forum/55854/a53-latencies-jitter-memory-access
23. Linux is now a RTOS. PREEMPT_RT Real-Time Kernel Support Finally Merged into Linux 6.12 : r/embedded - Reddit, accessed January 1, 2026, https://www.reddit.com/r/embedded/comments/1fmkojo/linux_is_now_a_rtos_preempt_rt_realtime_kernel/
24. A 16-nm Multiprocessing System-on-Chip Field-Programmable Gate Array Platform | Request PDF - ResearchGate, accessed January 1, 2026, https://www.researchgate.net/publication/301333455_A_16-nm_Multiprocessing_System-on-Chip_Field-Programmable_Gate_Array_Platform
25. 2.1. Release Notes — Processor SDK RTOS Documentation - http - Texas Instruments, accessed January 1, 2026, https://software-dl.ti.com/processor-sdk-rtos/esd/docs/06_01_00_08/rtos/index_release_specific.html
26. STATIC CHECKING OF INTERRUPT-DRIVEN SOFTWARE A Thesis Submitted to the Fa ulty of Purdue University by Dennis W. Brylow In Parti, accessed January 1, 2026, https://www.cs.mu.edu/~brylow/papers/Brylow-Dissertation2003.pdf
27. Operating system support for execution time budgets for thread groups - ResearchGate, accessed January 1, 2026, https://www.researchgate.net/publication/234802461_Operating_system_support_for_execution_time_budgets_for_thread_groups
28. Alternate scheduling - EVL, accessed January 1, 2026, https://evlproject.org/dovetail/kernel-api/altsched/index.html
29. AM6442: FIQ Interrupt on A53 - Processors forum - TI E2E, accessed January 1, 2026, https://e2e.ti.com/support/processors-group/processors/f/processors-forum/1164496/am6442-fiq-interrupt-on-a53
30. How PREEMPT_RT Improves Embedded Linux Reliability - SoftwareLogic, accessed January 1, 2026, https://softwarelogic.co/en/blog/how-preempt_rt-improves-embedded-linux-reliability
31. Intro to Real-Time Linux for Embedded Developers, accessed January 1, 2026, https://www.linuxfoundation.org/blog/blog/intro-to-real-time-linux-for-embedded-developers
32. An Introduction to real-time Linux | Ubuntu, accessed January 1, 2026, https://ubuntu.com/engage/an-introduction-to-real-time-linux
33. Real-Time System Benchmarking with Embedded Linux and RT Linux on a Multi-Core Hardware Platform, accessed January 1, 2026, https://liu.diva-portal.org/smash/get/diva2:1836643/FULLTEXT01.pdf
34. outrunner motor RPM / KV? - Motors, Mechanics, Power and CNC - Arduino Forum, accessed January 1, 2026, https://forum.arduino.cc/t/outrunner-motor-rpm-kv/611482
35. FPV Motors KV Rating Explained:What It Means & How to Choose - ligpower.com, accessed January 1, 2026, https://www.ligpower.com/blog/what-is-kv-rating-in-fpv-motors.html
36. Why do brushless motors have a kv rating? - Electrical Engineering Stack Exchange, accessed January 1, 2026, https://electronics.stackexchange.com/questions/262106/why-do-brushless-motors-have-a-kv-rating
37. Discussion How Does Kv Relate to Running RPM...??? - RC Groups, accessed January 1, 2026, https://www.rcgroups.com/forums/showthread.php?1876171-How-Does-Kv-Relate-to-Running-RPM
38. I just learned what Kv REALLY means! : r/radiocontrol - Reddit, accessed January 1, 2026, https://www.reddit.com/r/radiocontrol/comments/1xykjs/i_just_learned_what_kv_really_means/
39. The Physics of Drones: Understanding How They Fly - StickMan ..., accessed January 1, 2026, https://stickmanphysics.com/physics-of-drones/
40. Urban Wake Field Generation Using LES for Application to Quadrotor Flight - Carleton University, accessed January 1, 2026, https://carleton.ca/atarg/wp-content/uploads/Sutherland_MAScThesis_2015.pdf
41. Quad-Rotor Control Design Series - MTwallets, accessed January 1, 2026, https://www.mtwallets.com/timeout-quadrotor-project-why/
42. How yaw works? Drone yaw control - YouTube, accessed January 1, 2026, https://www.youtube.com/watch?v=aftM6NKKngA
43. Dynamics, Modeling, Simulation and Control of Mid-Flight Coupling of Quadrotors by Daniel Larsson A Thesis Presented in Partial - CORE, accessed January 1, 2026, https://core.ac.uk/download/pdf/79584375.pdf
44. Vibration Analysis Using Multi-Layer Perceptron Neural Networks for Rotor Imbalance Detection in Quadrotor UAV - MDPI, accessed January 1, 2026, https://www.mdpi.com/2504-446X/9/2/102
45. Introduction to Fast Fourier Transform (FFT) Analysis - Vibration Research, accessed January 1, 2026, https://vibrationresearch.com/blog/fast-fourier-transform-fft-analysis/
46. Betaflight Filtering 101 - Oscar Liang, accessed January 1, 2026, https://oscarliang.com/betaflight-filtering/
47. Nyquist–Shannon sampling theorem - Wikipedia, accessed January 1, 2026, https://en.wikipedia.org/wiki/Nyquist%E2%80%93Shannon_sampling_theorem
48. Question about the Nyquist-Shannon sampling theorem : r/DSP - Reddit, accessed January 1, 2026, https://www.reddit.com/r/DSP/comments/svulph/question_about_the_nyquistshannon_sampling_theorem/
49. Filtering and noise - Gyroflow, accessed January 1, 2026, https://old-docs.gyroflow.xyz/tech/filtering/
50. Derivation of Nyquist Frequency and Sampling Theorem [closed] - Signal Processing Stack Exchange, accessed January 1, 2026, https://dsp.stackexchange.com/questions/61174/derivation-of-nyquist-frequency-and-sampling-theorem
51. Unity Rotation Fixed: Euler Angles vs. Quaternions (Explained ..., accessed January 1, 2026, https://omitram.com/euler-angles-vs-quaternions-understanding-gimbal-lock-in-unity/
52. Euler vs Quaternion - What's the difference? - YouTube, accessed January 1, 2026, https://www.youtube.com/watch?v=sJcVJEOwLUs
53. Euler angles and gimbal lock - Mathematics Stack Exchange, accessed January 1, 2026, https://math.stackexchange.com/questions/8980/euler-angles-and-gimbal-lock
54. Quaternions, euler angles and the problem of gimble lock during interpolation of euler angles can be tricky to understand. This video explains the concept well. Hope it helps. : r/gamedev - Reddit, accessed January 1, 2026, https://www.reddit.com/r/gamedev/comments/15fv6ph/quaternions_euler_angles_and_the_problem_of/
55. From Complex Numbers to Quaternions - Jingnan Shi, accessed January 1, 2026, https://jingnanshi.com/blog/from_complex_numbers_to_quaternions.html
56. Quaternions and spatial rotation - Wikipedia, accessed January 1, 2026, https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
57. How Quaternions Produce 3D Rotation - Penguin Maths, accessed January 1, 2026, https://penguinmaths.blogspot.com/2019/06/how-quaternions-produce-3d-rotation.html
58. Head-of-line blocking - Glossary - MDN Web Docs, accessed January 1, 2026, https://developer.mozilla.org/en-US/docs/Glossary/Head_of_line_blocking
59. Difference between TCP and UDP? - Stack Overflow, accessed January 1, 2026, https://stackoverflow.com/questions/5970383/difference-between-tcp-and-udp
60. Comparative Analysis of QUIC and Other Network Protocols for Real-Time Robotic Control - Document Server@UHasselt, accessed January 1, 2026, https://documentserver.uhasselt.be/bitstream/1942/46864/2/4349c449-e468-45c5-81d8-23c8f4468331.pdf
61. Head-of-Line Blocking: Explanation and Designing a Flexible Network Layer - Medium, accessed January 1, 2026, https://medium.com/@aditimishra_541/head-of-line-blocking-explanation-and-designing-a-flexible-network-layer-a1b44cde01be
62. Recreational Flyers & Community-Based Organizations | Federal Aviation Administration, accessed January 1, 2026, https://www.faa.gov/uas/recreational_flyers
63. 14 CFR § 107.31 - Visual line of sight aircraft operation. - Cornell Law School, accessed January 1, 2026, https://www.law.cornell.edu/cfr/text/14/107.31
64. BVLOS & VLOS: A Comprehensive Guide for Drone Pilots, accessed January 1, 2026, https://www.dronepilotgroundschool.com/bvlos-vlos/
65. New Drone Laws in the USA [Updated in 2025], accessed January 1, 2026, https://www.thedroneu.com/blog/usa-drone-laws-regulations-by-state/
6. Remote Identification of Drones | Federal Aviation Administration, accessed January 1, 2026, https://www.faa.gov/uas/getting_started/remote_id

---

## Appendix A: The Rosetta Stone (Engineering vs. The Field)

This appendix provides a "translation matrix" to link graduate-level aerospace concepts with the common terminology used by FPV pilots and hobbyists.

| **Engineering Concept** | **Hobbyist / Pilot Slang** | **The Reality Connection** |
| :--- | :--- | :--- |
| **Internal Resistance ($R_{ir}$) & SEI Layer** | **"Voltage Sag"** or **"Puffing"** | When a pilot says, *"My battery sagged to 3.2V on a punch-out,"* they are empirically observing Ohm's Law: $V_{drop} = I \times R_{ir}$. A "puffed" LiPo is the SEI layer decomposing into gas due to thermal stress. |
| **Nyquist Aliasing ($f_{alias}$)** | **"Hot Motors"** or **"D-Term Heat"** | If motors come down hot, a pilot says, *"My D-Gains are too high."* In reality, high-frequency frame vibration is **aliasing** past the filter, appearing as low-frequency motion. The D-term fights this "ghost" motion, turning electricity into heat. |
| **Stochastic Jitter / Latency** | **"Desync"** or **"Looptime"** | Pilots obsess over "8k Looptime." If the drone randomly falls out of the sky during a flip, they call it a **"Desync."** In reality, the scheduler jitter likely exceeded the control period or the ESC lost synchronization. |
| **Quaternions vs. Euler Angles** | **"Gimbal Lock"** vs. **"Horizon Mode"** | A pilot flying in **"Angle Mode"** (Euler) cannot flip upside down without the controller fighting them. A pilot in **"Acro Mode"** (Quaternions/Rates) can spin indefinitely because they are controlling the derivative vector ($\dot{q}$). |
| **Bode Plot / Bandwidth** | **"Prop Wash"** | When a drone wobbles descending into its own air, pilots call it **"Prop Wash Oscillation."** An engineer sees a loss of **Control Authority** because turbulent air reduces the prop's aerodynamic gain ($K_{thrust}$). |
| **I-Term Windup** | **"Integrator Accumulation"** | Pilots know if they crash and leave the throttle up, the drone will try to spin to the moon. This is the **Integrator** accumulating error ($ \int e(t) dt $) because the drone is physically obstructed. |