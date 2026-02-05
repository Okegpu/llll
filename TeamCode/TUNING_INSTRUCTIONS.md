Ra# Shooter Tuning Instructions

## Goal
Calibrate the dual-flywheel shooter to automatically adjust its speed based on the distance to the AprilTag.

## Controls
**Tuning Mode is active only when you HOLD the `Right Bumper` (RB).**

| Action | Control (While Holding RB) |
| :--- | :--- |
| **Increase Base Velocity** | D-Pad **UP** |
| **Decrease Base Velocity** | D-Pad **DOWN** |
| **Increase Sensitivity** | D-Pad **RIGHT** |
| **Decrease Sensitivity** | D-Pad **LEFT** |
| **SAVE Settings** | Press **A** (while holding RB) |

---

## Tuning Process

### 1. Mid-Distance (Base Velocity)
1.  Position the robot at a **medium shooting distance** (where the Limelight `AutoTy` is close to 0).
2.  Hold `RB` and use **D-Pad Up/Down** to adjust `AutoBase`.
3.  Test shots until they land in the goal consistently.

### 2. Distance Variation (Sensitivity)
1.  Move the robot **closer** or **farther** away.
2.  If the shots are **too low** (speed too slow) or **too high** (speed too fast), you need to adjust Sensitivity.
3.  Hold `RB` and use **D-Pad Left/Right** to adjust `AutoSens`.
    *   *Higher Sensitivity* = Speed changes *more* drastically with distance.
    *   *Lower Sensitivity* = Speed changes *less* with distance.

### 3. Save
1.  Once shots are consistent at multiple distances, **Hold RB and Press A**.
2.  Look for **"Config: SAVED"** on the Driver Station telemetry.
3.  Your settings are now permanent and will load automatically next time.

---

## Telemetry Guide
*   **AutoBase:** The starting speed (at mid-distance).
*   **AutoSens:** How much the speed adjusts per degree of distance.
*   **RawCalcVel:** The calculated motor velocity target (useful to see if the math is trying to go beyond the motor's max speed).
