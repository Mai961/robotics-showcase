// Simplified from the robot library's swerve/sim/SwerveModuleIOSimpleSim.java. Removed: the
// controller re-tuning stubs, logging and the duplicated sample fields. Illustrative, not
// compilable; the full implementation is not public.
//
// The swerve modules in simulation: each module simply IS in the state it was commanded to
// (wheel speed and steering angle), and the drive distance is integrated from that speed.
// This is the module simulation the robot program wires in when not running on the robot.

public class SwerveModuleIOSimpleSim implements SwerveModuleIO {
    private final SwerveSimConfig config;                                      // wheel diameter
    private final SwerveModuleState commanded = new SwerveModuleState();     // m/s, angle
    private final SwerveModulePosition position = new SwerveModulePosition(); // m, angle
    private double previousTimestamp = Timer.getTimestamp();

    /** Integrate the commanded speed into distance, and report it as motor angles (rad). */
    @Override
    public void updateInputs(SwerveModuleIOInputs data) {
        double dt = Timer.getTimestamp() - previousTimestamp;
        previousTimestamp += dt;

        position.distanceMeters += commanded.speedMetersPerSecond * dt;
        position.angle = commanded.angle;
        double steerVelocity =
                dt > 0 ? (position.angle.getRadians() - data.steerMotorPositionRad) / dt : 0.0;

        // Wheel distance -> wheel angle: angle = distance / radius = distance * 2 / diameter.
        double diameter = config.wheelDiameter.in(Meter);
        data.driveMotorPositionRad = position.distanceMeters * 2.0 / diameter;
        data.driveMotorVelocityRadPerSec = commanded.speedMetersPerSecond * 2.0 / diameter;
        data.steerMotorPositionRad = position.angle.getRadians();
        data.steerMotorVelocityRadPerSec = steerVelocity;
    }

    /** Open loop: speed proportional to the share of battery voltage, 4 m/s at full voltage. */
    @Override
    public void setDriveOpenLoop(Voltage volts) {
        commanded.speedMetersPerSecond =
                volts.in(Volts) / RobotController.getBatteryVoltage() * 4.0;
    }

    /** Closed loop: the commanded wheel speed is reached immediately. */
    @Override
    public void setDriveVelocity(LinearVelocity wheelSpeed) {
        commanded.speedMetersPerSecond = wheelSpeed.in(MetersPerSecond);
    }

    /** The commanded steering angle is reached immediately. */
    @Override
    public void setSteerAngleAbsolute(Angle angle) {
        commanded.angle = new Rotation2d(angle);
        position.angle = commanded.angle;
    }
}
