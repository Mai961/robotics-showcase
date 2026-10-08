// Simplified from the robot library's io/MotorIOSim.java. Removed: duty-cycle and position-reset
// setters, profile re-configuration and the (commented-out) soft limits.
// Illustrative, not compilable; the full implementation is not public.
//
// A simple kinematic motor simulation that moves directly according to its limits. This avoids
// the complexity and potential bugs of a full physics simulation when realism is not required.
// (Verbatim from the source's class comment.) There is no load, no friction and no current: a
// velocity command is reached instantly, and a position command follows a trapezoid profile.

public class MotorIOSim implements MotorIO {
    private enum Mode { VOLTAGE, VELOCITY, POSITION }

    private Mode mode = Mode.VOLTAGE;
    private double positionRot, velocityRps, appliedVolts, velocitySetpointRps;
    private TrapezoidProfile.State profileState = new TrapezoidProfile.State(0, 0);
    private TrapezoidProfile.State goal = new TrapezoidProfile.State(0, 0);
    private double lastTimestamp = Timer.getTimestamp();
    private final double maxVelocityRps;
    private final TrapezoidProfile profile;

    public MotorIOSim(SubsystemConfig cfg) {
        // Top speed: one Kraken X60's free speed through the gear ratio. Without load, the number
        // of motors does not change it.
        maxVelocityRps = DCMotor.getKrakenX60Foc(1).freeSpeedRadPerSec / (2 * Math.PI)
                / cfg.simConfig.gearRatio;
        // Position moves use the configured profile, or 50 rot/s and 100 rot/s^2 by default.
        profile = new TrapezoidProfile(cfg.simConfig.profile != null
                ? cfg.simConfig.profile
                : new TrapezoidProfile.Constraints(50.0, 100.0));
    }

    @Override
    public void readInputs(MotorInputs inputs) {
        double dt = Timer.getTimestamp() - lastTimestamp;
        lastTimestamp += dt;

        switch (mode) {
            case VOLTAGE:   // speed proportional to voltage, 12 V = free speed
                velocityRps = (appliedVolts / 12.0) * maxVelocityRps;
                positionRot += velocityRps * dt;
                profileState = new TrapezoidProfile.State(positionRot, velocityRps);
                break;
            case VELOCITY:  // the setpoint is reached immediately, capped at free speed
                velocityRps = MathUtil.clamp(velocitySetpointRps, -maxVelocityRps, maxVelocityRps);
                positionRot += velocityRps * dt;
                profileState = new TrapezoidProfile.State(positionRot, velocityRps);
                break;
            case POSITION:  // follow a trapezoid profile toward the goal
                profileState = profile.calculate(dt, profileState, goal);
                positionRot = profileState.position;
                velocityRps = profileState.velocity;
                break;
        }

        inputs.positionRot = positionRot;
        inputs.velocityRotPerSecond = velocityRps;
        inputs.appliedVolts = appliedVolts;
        inputs.motorVolts = appliedVolts;
        inputs.currentStatorAmps = 0.0;  // a kinematic model has no current
        inputs.currentSupplyAmps = 0.0;
    }

    @Override
    public void setVoltage(double voltage) {
        mode = Mode.VOLTAGE;
        appliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
    }

    @Override
    public void setVelVoltSetpoint(AngularVelocity velocity, Voltage feedForwardVoltage) {
        mode = Mode.VELOCITY;  // the kinematic model ignores the feedforward voltage
        velocitySetpointRps = velocity.in(RotationsPerSecond);
    }

    @Override
    public void setMotionMagicSetpoint(Angle position) {
        mode = Mode.POSITION;
        goal = new TrapezoidProfile.State(position.in(Rotations), 0.0);
    }
}
