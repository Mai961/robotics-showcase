// Simplified from the robot library's io/MotorIO.java. Removed: the duty-cycle, soft-limit,
// current-limit, neutral-mode and re-zeroing methods. Illustrative, not compilable; the full
// implementation is not public.
//
// The hardware boundary. A subsystem (the turret, the shooter...) talks only to this interface. On
// the robot it is backed by MotorIOTalonFX; in simulation by MotorIOSim. Nothing above this line
// knows which one it got. Units are mechanism units: rotations of the mechanism, not of the motor.

/** The subsystem owns its IO and logs its inputs every loop. */
public interface MotorIO {
    /** Copy the latest sensor values into `inputs`; the subsystem logs them every loop. */
    default void readInputs(MotorInputs inputs) {}

    /** Open loop, volts. */
    default void setVoltage(double voltage) {}

    /** Closed-loop velocity on the motor controller, plus a feedforward voltage added on top. */
    default void setVelVoltSetpoint(AngularVelocity velocity, Voltage feedForwardVoltage) {}

    /** Closed-loop position along a motion profile that runs on the motor controller. */
    default void setMotionMagicSetpoint(Angle position) {}

    /** Change the controller gains (kP, kI, kD, kS, kV, kA) at runtime. */
    default void updateGains(Slot0Configs slot0) {}
}
