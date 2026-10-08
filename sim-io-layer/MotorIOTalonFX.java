// Simplified from the robot library's io/MotorIOTalonFX.java. Removed: the configuration of
// current limits, soft limits, inversion and remote CANcoder feedback, follower motors, retry
// helpers and the remaining setters. Illustrative, not compilable; the full implementation is not
// public.
//
// The real implementation: a CTRE TalonFX (Kraken/Falcon motor controller, Phoenix 6 API). The
// closed loops run ON the motor controller; the robot loop only sends setpoints.

public class MotorIOTalonFX implements MotorIO {
    private final TalonFX motor;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0).withEnableFOC(true);
    private final MotionMagicVoltage motionMagicRequest =
            new MotionMagicVoltage(0.0).withEnableFOC(true);
    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;

    public MotorIOTalonFX(SubsystemConfig cfg) {
        motor = new TalonFX(cfg.mainId, cfg.mainBus);
        // The gear ratio lives on the controller, so readings and setpoints are in mechanism units.
        cfg.fxConfig.Feedback.SensorToMechanismRatio = cfg.SensorToMechanismRatio;
        motor.getConfigurator().apply(cfg.fxConfig);

        position = motor.getPosition();
        velocity = motor.getVelocity();
        // 100 Hz: nothing reads these faster than the 50 Hz loop; the previous 1000 Hz per motor
        // was the main CANivore bus-load driver (~80% utilization measured).
        BaseStatusSignal.setUpdateFrequencyForAll(100.0, position, velocity);
        motor.optimizeBusUtilization();
    }

    @Override
    public void readInputs(MotorInputs inputs) {
        inputs.positionRot = position.getValueAsDouble();
        inputs.velocityRotPerSecond = velocity.getValueAsDouble();
        inputs.motorVolts = motor.getMotorVoltage().getValueAsDouble();
        inputs.appliedVolts = motor.getSupplyVoltage().getValueAsDouble();  // as in the source
        inputs.currentStatorAmps = motor.getStatorCurrent().getValueAsDouble();
        inputs.currentSupplyAmps = motor.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setControl(new VoltageOut(voltage));
    }

    @Override
    public void setVelVoltSetpoint(AngularVelocity velocity, Voltage feedForwardVoltage) {
        motor.setControl(
                velocityRequest.withVelocity(velocity).withFeedForward(feedForwardVoltage));
    }

    @Override
    public void setMotionMagicSetpoint(Angle position) {
        motor.setControl(motionMagicRequest.withPosition(position));
    }
}
