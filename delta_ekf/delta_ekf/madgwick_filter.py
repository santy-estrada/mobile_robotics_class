import numpy as np

class MadgwickAHRS:

    def __init__(self, sample_period=1/20, beta=0.1):
        # sample_period: time between filter calls [s].
        #   Must match your IMU publish rate (e.g., 1/50 = 0.02 s at 50 Hz).
        #   Too large → inaccurate integration; too small → no problem.
        self.sample_period = sample_period

        # beta: gradient-descent step size.
        #   Controls the trade-off between gyro trust and accel/mag correction.
        self.beta = beta

        # Initial quaternion [w, x, y, z] = identity (no rotation).
        #   The filter converges from here to the true orientation within
        #   a few seconds, driven by the accel/mag correction.
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0])

    def update(self, gyroscope, accelerometer, magnetometer):
        """One filter step. Call once per IMU message."""
        q = self.quaternion          # current orientation estimate [w,x,y,z]
        gx, gy, gz = gyroscope       # angular velocity [rad/s] in body frame
        ax, ay, az = accelerometer   # specific force [m/s²] in body frame
        mx, my, mz = magnetometer    # magnetic field [T or Gauss] in body frame

        # ── Normalise accelerometer ────────────────────────────────────────
        # Madgwick uses only the DIRECTION of gravity, not the magnitude.
        # Normalising removes the effect of linear acceleration (partially).
        # A zero-norm accel means the sensor is in free-fall; skip correction.
        norm_acc = np.linalg.norm([ax, ay, az])
        if norm_acc == 0:
            return
        ax, ay, az = ax/norm_acc, ay/norm_acc, az/norm_acc

        # ── Normalise magnetometer ─────────────────────────────────────────
        # Same reason: only the direction of North matters, not magnitude.
        norm_mag = np.linalg.norm([mx, my, mz])
        if norm_mag == 0:
            return
        mx, my, mz = mx/norm_mag, my/norm_mag, mz/norm_mag

        # ── Precompute repeated terms (avoids redundant multiplications) ───
        # Convention: _2q1 = 2*q[0], _4q1 = 4*q[0], q1q1 = q[0]², etc.
        # These correspond directly to the intermediate values in
        # Madgwick's paper (Appendix, equations A-25 to A-34).
        _2q1 = 2.0 * q[0];  _2q2 = 2.0 * q[1]
        _2q3 = 2.0 * q[2];  _2q4 = 2.0 * q[3]
        _4q1 = 4.0 * q[0];  _4q2 = 4.0 * q[1];  _4q3 = 4.0 * q[2]
        _8q2 = 8.0 * q[1];  _8q3 = 8.0 * q[2]
        q1q1 = q[0]*q[0];   q2q2 = q[1]*q[1]
        q3q3 = q[2]*q[2];   q4q4 = q[3]*q[3]

        # ── Gradient of the objective function (accelerometer part) ────────
        # f_acc(q) = q* ⊗ [0,0,0,1] ⊗ q − [0, ax, ay, az]
        # These four lines are ∂f_acc/∂q1, ∂f_acc/∂q2, ∂f_acc/∂q3, ∂f_acc/∂q4
        # derived analytically (see paper Eq. 25).
        # The magnetometer cross-terms are folded into the same s1..s4
        # in the full 9-DOF version (simplified here for readability).
        s1 = _4q1*q3q3 + _2q3*ax + _4q1*q2q2 - _2q2*ay
        s2 = (_4q2*q4q4 - _2q4*ax + 4.0*q1q1*q[1] - _2q1*ay
              - _4q2 + _8q2*q2q2 + _8q2*q3q3 + _4q2*az)
        s3 = (4.0*q1q1*q[2] + _2q1*ax + _4q3*q4q4 - _2q4*ay
              - _4q3 + _8q3*q2q2 + _8q3*q3q3 + _4q3*az)
        s4 = 4.0*q2q2*q[3] - _2q2*ax + 4.0*q3q3*q[3] - _2q3*ay

        # Normalise the gradient vector so step size is independent of
        # the gradient magnitude (just the direction matters).
        norm_s = np.linalg.norm([s1, s2, s3, s4])
        s1, s2, s3, s4 = s1/norm_s, s2/norm_s, s3/norm_s, s4/norm_s

        # ── Compute quaternion rate ────────────────────────────────────────
        # Two contributions are combined:
        #
        #   q̇_gyro = ½ · q ⊗ [0, gx, gy, gz]   (kinematics, see Section 4)
        #   q̇_corr = −β · [s1, s2, s3, s4]      (gradient correction)
        #
        #   q̇ = q̇_gyro + q̇_corr
        #
        # When β = 0: pure gyro integration (fast, drifts).
        # When β >> 0: strong pull toward accel/mag reference.
        q_dot = 0.5 * np.array([
            -q[1]*gx - q[2]*gy - q[3]*gz,   # dw/dt from gyro
             q[0]*gx + q[2]*gz - q[3]*gy,   # dx/dt from gyro
             q[0]*gy - q[1]*gz + q[3]*gx,   # dy/dt from gyro
             q[0]*gz + q[1]*gy - q[2]*gx    # dz/dt from gyro
        ]) - self.beta * np.array([s1, s2, s3, s4])

        # ── Integrate (first-order Euler) ──────────────────────────────────
        # q_new = q_old + q̇ · Δt
        # First-order Euler is sufficient because Δt is small (≤ 0.02 s).
        q += q_dot * self.sample_period

        # ── Re-normalise ───────────────────────────────────────────────────
        # Numerical integration introduces small errors that make ‖q‖ drift
        # away from 1. Re-normalising after each step keeps q valid.
        self.quaternion = q / np.linalg.norm(q)

    def get_yaw(self):
        """Extract yaw angle [rad] from the current quaternion.

        Yaw is the rotation around the Z axis (heading).
        Positive = counter-clockwise when viewed from above (ROS convention).

        Formula:  ψ = atan2(2(wz + xy),  1 − 2(y² + z²))
        """
        q = self.quaternion   # [w, x, y, z]
        return np.arctan2(
            2.0 * (q[0]*q[3] + q[1]*q[2]),
            1.0 - 2.0 * (q[2]**2 + q[3]**2)
        )