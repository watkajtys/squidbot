# The Squid Drone Study Guide & Lecture Series
**Academic Standard:** M.Sc. Robotics / PhD Candidate Prep
**Philosophy:** "First we prove it (Math), then we build it (Lab)."

This document serves as both a syllabus and a detailed study guide. It maps the theoretical concepts to specific academic resources, textbooks, and papers. While the *Labs* (`COURSE.md`) focus on getting things to fly, this guide focuses on *why* they fly.

---

## **PHASE I: FOUNDATIONS (The Physics)**

### **Lecture 0: Systems Engineering**
#### Core Readings
*   **Real-Time Systems:** "Real-Time Systems" by Jane W. S. Liu (Chapter 6: RMS vs EDF). *(Classic Text)*.
*   **Signals & Systems:** "Signals and Systems" by Oppenheim & Willsky (Chapter 7: Sampling). *(Classic Text. Free Alternative: MIT OCW 6.003)*.
*   **Case Study:** "What Really Happened on Mars?" (The Pathfinder priority inversion). *(Free Online)*.

#### Key Concepts
*   **0.1 The Real-Time Constraint:**
    *   **Scheduling Theory:** Rate Monotonic Scheduling (RMS) vs. Earliest Deadline First (EDF).
    *   **The Mars Pathfinder Bug:** Priority Inversion and the need for Priority Inheritance protocols.
    *   **Latency Analysis:** Interrupt Service Routine (ISR) overhead vs. Context Switching costs.
*   **0.2 Nyquist-Shannon Sampling Theorem:**
    *   **The Proof:** Convolution in the frequency domain.
    *   **Aliasing:** How high frequencies "fold" back ($f_{alias} = |f_{signal} - N \cdot f_{sample}|$).
    *   **Filter Design:** Butterworth vs. Chebyshev analog filters (Anti-aliasing hardware).
*   **0.3 Discrete Time Physics:**
    *   **Numerical Integration:** Forward Euler ($O(h)$) vs. Trapezoidal ($O(h^2)$) vs. Runge-Kutta 4 ($O(h^4)$).
    *   **Symplectic Integrators:** Preserving energy (Hamiltonian) in simulation over long durations.
*   **0.4 Jitter & Determinism:**
    *   **The Jitter Problem:** Why a steady 20ms loop is superior to a 5ms loop that occasionally jumps to 50ms.
    *   **Clock Skew:** Handling the difference between the drone's clock and the laptop's clock.
*   **0.5 The Filter-Delay Tradeoff:**
    *   **Phase Lag:** The mathematical law that says every filter adds time-delay.
    *   **Group Delay:** Why some frequencies are delayed more than others, distorting the "shape" of the signal.

### **Lecture 1: Embedded Communication & Numerics**
#### Core Readings
*   **Signal Integrity:** "High-Speed Digital Design: A Handbook of Black Magic" by Johnson & Graham. *(Industry Standard)*.
*   **CRC:** "A Painless Guide to CRC Error Detection Algorithms" by Ross Williams. *(Free Online)*.

#### Key Concepts
*   **1.1 Signal Integrity:**
    *   **Transmission Lines:** Characteristic Impedance ($Z_0 = \sqrt{L/C}$).
    *   **Reflections:** The Reflection Coefficient ($\\Gamma = \frac{Z_L - Z_0}{Z_L + Z_0}$).
    *   **Rise Time:** The "Knee Frequency" ($f_{knee} \approx \frac{0.5}{t_{rise}}$).
*   **1.2 Information Theory:**
    *   **Hamming Distance:** Minimum bit flips to transform one valid code word to another.
    *   **CRC Math:** Polynomial long division over $GF(2)$. Selection of generator polynomials for burst error detection.
*   **1.3 Numerical Linear Algebra:**
*   **1.4 The Anatomy of a Packet:**
    *   **Hexadecimal Literacy:** Reading `0x` values.
    *   **Endianness:** Big-Endian vs. Little-Endian (Which end of the number is first?).
    *   **The Checksum:** The XOR operator as a primitive error detector.
    *   **Floating Point:** IEEE 754 representation, Machine Epsilon, and Catastrophic Cancellation.
    *   **Matrix Decomposition:** 
        *   **LU:** For solving linear systems.
        *   **Cholesky:** For symmetric positive-definite matrices (Covariance matrices).
        *   **SVD:** For determining the Rank and Condition Number ($\\kappa(A) = \frac{\\sigma_{max}}{\\sigma_{min}}$).

---

## **PHASE II: OBSERVABILITY (The Math of Sensing)**

### **Lecture 2: Stochastic Processes**
#### Core Readings
*   **Probability:** "Probabilistic Robotics" by Thrun, Burgard, Fox (Chapter 3: Gaussian Filters). *(Standard Text)*.
*   **IMU Noise:** "IEEE Standard Specification Format Guide and Test Procedure for Single-Axis Interferometric Fiber Optic Gyros" (IEEE Std 952-1997).

#### Key Concepts
*   **2.1 Probability Review:**
    *   **Bayes' Theorem:** $P(x|z) = \frac{P(z|x)P(x)}{P(z)}$.
    *   **Marginalization:** Integrating out nuisance variables ($P(x) = \int P(x,y) dy$).
*   **2.2 Gaussian Distributions:**
    *   **Multivariate Normal:** Definition via Mean vector $\\mu$ and Covariance Matrix $\\Sigma$.
    *   **Mahalanobis Distance:** $d = \sqrt{(x-\\mu)^T \\Sigma^{-1} (x-\\mu)}$ (The "Standard Deviation" in 3D).
    *   **The Central Limit Theorem:** Why noise tends to be Gaussian.
*   **2.3 Sensor Noise Models:**
    *   **Power Spectral Density (PSD):** Characterizing noise vs. frequency.
    *   **Allan Variance:** Distinguishing between Angle Random Walk (White Noise) and Rate Random Walk (Bias Drift).

### **Lecture 3: Computer Vision Fundamentals**
#### Core Readings
*   **Geometry:** "Multiple View Geometry in Computer Vision" by Hartley & Zisserman (Chapters 2 & 9). *(Standard Text)*.

#### Key Concepts
*   **3.1 Projective Geometry:**
    *   **Coordinate Frames:** World ($W$) $\to$ Camera ($C$) $\to$ Image Plane ($I$).
    *   **Global Systems:** Geodetic (LLA) vs. Cartesian (NED/ECEF) for GPS navigation.
    *   **Homogeneous Coordinates:** Representing points at infinity.
    *   **The Extrinsic Matrix:** $[R | t] \in SE(3)$.
*   **3.2 The Intrinsic Matrix ($K$):**
    *   **Pinhole Model:** $u = f_x \frac{X}{Z} + c_x$, $v = f_y \frac{Y}{Z} + c_y$.
    *   **Skew:** Handling non-rectangular pixels.
*   **3.3 Multi-View Geometry:**
    *   **Epipolar Geometry:** The Essential Matrix ($E = [t]_\times R$) and Fundamental Matrix ($F$).
    *   **Triangulation:** Solving $AX=0$ via SVD to find 3D point depth.

---

## **PHASE III: RIGID BODY DYNAMICS (The Math of Motion)**

### **Lecture 4: Rotations & Lie Groups (Part I)**
#### Core Readings
*   **Lie Theory:** "A Mathematical Introduction to Robotic Manipulation" by Murray, Li, Sastry (Chapter 2). *(Free PDF available online)*.
*   **Quaternions:** "Quaternion kinematics for the error-state Kalman filter" by Joan Solà. *(Free via arXiv)*.

#### Key Concepts
*   **4.1 The Rotation Group $SO(3)$:**
    *   **Definition:** Orthogonal matrices with determinant +1 ($R^TR = I$).
    *   **Limitations:** The Singularity problem (Gimbal Lock) of Euler Angles.
*   **4.2 Quaternions:**
    *   **Algebra:** Hamilton Product and Conjugates.
    *   **Rotation:** $v' = q v q^{-1}$.
    *   **SLERP:** Spherical Linear Interpolation for smooth animation.
*   **4.3 The Lie Algebra $\\mathfrak{so}(3)$:**
    *   **Skew-Symmetric Matrices:** The "Hat" operator ($\\omega \to [\\omega]_\times$).
    *   **Rodrigues' Formula:** The closed-form Exponential Map ($\\exp: \\mathfrak{so}(3) \\to SO(3)$).

### **Lecture 5: Control Theory**
#### Core Readings
*   **Control:** "Feedback Control of Dynamic Systems" by Franklin, Powell, Emami-Naeini (Chapters 4 & 5).

#### Key Concepts
*   **5.1 Frequency Domain Analysis:**
    *   **Laplace Transform properties:** Differentiation becomes multiplication by $s$.
    *   **Bode Plots:** Gain Margin and Phase Margin analysis.
*   **5.2 Root Locus:**
    *   Tracing closed-loop poles as gain $K$ varies.
    *   Determining instability (poles crossing into Right-Half Plane).
*   **5.3 Lyapunov Stability:**
    *   **Definition:** Stability in the sense of Lyapunov (ISL) vs. Asymptotic Stability.
    *   **Direct Method:** Finding a scalar energy function $V(x)$ such that $V(x) > 0$ and $\\dot{V}(x) < 0$.
    *   **LaSalle's Invariance Principle:** Handling cases where $\\dot{V}(x) = 0$.
*   **5.4 Poles, Zeros, and Damping:**
    *   **The Transfer Function:** Treating the drone as a \"Black Box\" math equation.
    *   **Damping Ratio ($\\zeta$):** 
        *   **Underdamped ($\\zeta < 1$):** The drone overshoots and bounces (oscillates).
        *   **Overdamped ($\\zeta > 1$):** The drone is \"lazy\" and takes too long to reach the target.
        *   **Critically Damped ($\\zeta = 1$):** The \"Goldilocks\" zone. Fastest return to target with ZERO bounce.


---

## **PHASE IV: ESTIMATION (The Math of Belief)**

### **Lecture 6 & 7: State Estimation (The Kalman Filter)**
#### Core Readings
*   **Estimation:** "Estimation with Applications to Tracking and Navigation" by Bar-Shalom & Li (Chapters 5 & 10). *(Standard Text)*.
*   **Robotics:** "State Estimation for Robotics" by Timothy Barfoot (Chapter 3). *(Free PDF available online)*.

#### Key Concepts
*   **6.1 Linear Time-Invariant (LTI) Systems:**
    *   **Matrix Exponential:** Solving $\\dot{x} = Ax$ using $e^{At}$.
    *   **Discretization:** Converting $A$ (Continuous) to $F$ (Discrete) via Van Loan's method.
*   **6.2 Observability:**
    *   **Observability Matrix:** $\\mathcal{O} = [C^T, (CA)^T, ... (CA^{n-1})^T]^T$.
    *   **Rank Condition:** Why you can't estimate Yaw from a single GPS measurement.
*   **7.1 Optimality Proof:**
    *   Deriving KF as the recursive solution to the Linear Least Squares problem.
    *   **Orthogonality Principle:** The error is orthogonal to the measurements.
*   **7.2 The Algorithm:**
    *   **Prediction:** $\\hat{x}_{k|k-1} = F \\hat{x}_{k-1}$, $P_{k|k-1} = F P_{k-1} F^T + Q$.
    *   **Kalman Gain:** $K_k = P_{k|k-1} H^T (H P_{k|k-1} H^T + R)^{-1}$.
    *   **Update:** $\\hat{x}_k = \\hat{x}_{k|k-1} + K_k(z_k - H\\hat{x}_{k|k-1})$.
    *   **Covariance Update:** $P_k = (I - K_k H) P_{k|k-1}$ (Joseph Form for numerical stability).

### **Lecture 8: Optimization Theory (The Engine of VIO)**
#### Key Concepts
*   **8.1 The Non-Linear Least Squares Problem:**
    *   Cost Function: $J(x) = \frac{1}{2} \\sum \\| r_i(x) \\|^2$.
    *   Jacobian Matrix: $J_{ij} = \frac{\\partial r_i}{\\partial x_j}$.
*   **8.2 Solvers:**
    *   **Gauss-Newton:** Approximating the Hessian as $H \approx J^T J$.
    *   **Levenberg-Marquardt:** Adaptive damping ($\\lambda$) to interpolate between Gauss-Newton and Gradient Descent.
    *   **Schur Complement:** Efficiently solving sparse linear systems (marginalizing out landmarks).

---

## **PHASE V: PERCEPTION & MAPPING (The Math of Space)**

### **Lecture 9: SLAM & Geometry**
#### Core Readings
*   **Mapping:** "Probabilistic Robotics" by Thrun (Chapter 9). *(Standard Text)*.
*   **Optimization:** "Factor Graphs for Robot Perception" by Frank Dellaert. *(Free PDF available online)*.

#### Key Concepts
*   **9.1 ICP variants:**
    *   **Point-to-Point** vs. **Point-to-Plane** error metrics.
    *   **Outlier Rejection:** RANSAC (Random Sample Consensus).
*   **9.2 Occupancy Grids:**
    *   **Log-Odds:** $L(m|z) = L(m|z_{t-1}) + L(z|m) - L_0$. Avoiding floating point underflow.
    *   **Ray Casting:** Bresenham's Line Algorithm in 3D.

### **Lecture 10: Trajectory Generation**
#### Core Readings
*   **Differential Flatness:** "Minimum Snap Trajectory Generation and Control for Quadrotors" by Mellinger and Kumar (ICRA 2011). *(Free PDF available online)*.

#### Key Concepts
*   **10.1 Differential Flatness:**
    *   Mapping output space ($x, y, z, \\psi$) derivatives to input space (TotalThrust, $\\omega_x, \\omega_y, \\omega_z$).
    *   This proves any smooth path in 3D is flyable by a quadrotor.
*   **10.2 Quadratic Programming (QP):**
    *   Standard Form: $\\min \\frac{1}{2}x^T Q x + c^T x$ subject to $Ax \\leq b$.
    *   **Minimum Snap:** Constructing the $Q$ matrix from the 4th derivative of polynomials.

---

## **PHASE VI: LEARNING & INTELLIGENCE (The Math of AI)**

### **Lecture 11: Reinforcement Learning**
#### Core Readings
*   **Foundations:** "Reinforcement Learning: An Introduction" by Sutton & Barto. *(Free Draft available online)*.
*   **Algorithm:** "Proximal Policy Optimization Algorithms" by Schulman et al. (OpenAI). *(Free via arXiv)*.

#### Key Concepts
*   **11.1 Markov Decision Processes (MDP):**
    *   Tuple $(S, A, P, R, \\gamma)$.
    *   **Value Iteration** vs. **Policy Iteration**.
*   **11.2 Policy Gradients:**
    *   **Log-Derivative Trick:** $\\nabla_\theta J(\\theta) = E[\\nabla_\theta \\log \\pi_\theta(a|s) R]$.
    *   **The Baseline:** Reducing variance by subtracting Value function $V(s)$ (Advantage function).
*   **11.3 PPO (Proximal Policy Optimization):**
    *   **Clipped Surrogate Objective:** $L^{CLIP}(\\theta) = E[\\min(r_t(\\theta)\\hat{A}_t, \\text{clip}(r_t(\\theta), 1-\\epsilon, 1+\\epsilon)\\hat{A}_t)]$.
    *   **KL Divergence:** Preventing the policy from changing too fast (Trust Region).

### **Lecture 12: Differential Games**
#### Core Readings
*   **Guidance:** "Tactical and Strategic Missile Guidance" by Paul Zarchan (Chapter 2). *(Standard Text)*.

#### Key Concepts
*   **12.1 Pursuit-Evasion:**
    *   **Apollonius Circle:** The set of all points the pursuer can reach faster than the evader.
*   **12.2 Optimal Control:**
    *   **Hamilton-Jacobi-Bellman (HJB) Equation:** The continuous-time version of the Bellman equation.
    *   **Pontryagin's Minimum Principle:** Necessary conditions for optimality.

---

## **PHASE VII: ADVANCED TOPICS (The Frontier)**

### **Lecture 13: Visual Inertial Odometry (VIO)**
**"The Holy Grail of Navigation."**
#### Core Readings
*   **Pre-Integration:** "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry" by Forster et al. (IEEE TRO 2016). *(Free via IEEE/arXiv)*.
*   **Lie Theory:** "A micro Lie theory for state estimation in robotics" by Joan Solà. *(Free via arXiv)*.

#### Key Concepts
*   **13.1 Lie Groups (Part II): The Manifold Structure**
    *   **$SE(3)$:** The Special Euclidean Group (Rigid body transformations).
    *   **Tangent Space:** The Lie Algebra $\\mathfrak{se}(3)$ (Twists: $v \\in \\mathbb{R}^3, \\omega \\in \\mathbb{R}^3$).
    *   **The Logarithm Map:** $\\log: SE(3) \\to \\mathfrak{se}(3)$.
*   **13.2 Manifold Optimization:**
    *   **The Challenge:** You cannot do $x_{new} = x_{old} + \\Delta x$ because rotations don't add linearly.
    *   **The Solution (Retraction):** $x_{new} = x_{old} \\boxplus \\Delta x = x_{old} \\cdot \\exp(\\Delta x)$.
    *   **Error State:** Defining error in the tangent space $\\delta x = x \\boxminus \\hat{x} = \\log(\\hat{x}^{-1} x)$.
*   **13.3 IMU Pre-Integration (Forster et al.):**
    *   **Problem:** Re-linearizing the IMU trajectory every time the bias changes is too slow.
    *   **Solution:** Integrating IMU measurements in a *relative* frame that is independent of the initial state.
    *   **Jacobians:** Propagating noise covariance through the non-linear integration.
*   **13.4 Factor Graphs:**
    *   **Bipartite Graph:** Variable nodes (Poses, Landmarks) connected to Factor nodes (Constraints/Measurements).
    *   **Smoothing:** Solving for the *entire* history (Full Smoothing) vs. Fixed-Lag Smoothing (Sliding Window).

### **Lecture 14: Swarm Theory**
#### Core Readings
*   **Graph Theory:** "Graph Theoretic Methods in Multiagent Networks" by Mesbahi and Egerstedt. *(Free PDF available online)*.

#### Key Concepts
*   **14.1 Algebraic Graph Theory:**
    *   **The Laplacian Matrix:** $L = D - A$.
    *   **Fiedler Eigenvalue:** The second smallest eigenvalue $\\lambda_2$ determines the speed of consensus convergence.
*   **14.2 Distributed Control:**
    *   **Consensus Protocol:** $\\dot{x}_i = \\sum_{j \\in N_i} (x_j - x_i)$.
    *   **Safety Barriers:** Control Barrier Functions (CBF) for collision avoidance.

### **Lecture 15: Deep Perception**
#### Core Readings
*   **Metrics:** "Siamese Neural Networks for One-shot Image Recognition" (Koch et al.). *(Free via arXiv)*.

#### Key Concepts
*   **15.1 Convolutional Neural Networks (CNNs):**
    *   **Receptive Fields:** How convolution captures local spatial features.
    *   **Backpropagation:** The Chain Rule applied to tensor graphs.
*   **15.2 Metric Learning:**
    *   **Siamese Networks:** Learning to match image patches for loop closure detection.
    *   **Triplet Loss:** Minimizing intra-class distance while maximizing inter-class distance.
