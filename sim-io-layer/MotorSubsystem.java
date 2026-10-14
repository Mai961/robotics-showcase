// Simplified from the robot library's subsystem/MotorSubsystem.java. Removed: the other commands
// (duty cycle, stop, current limits) and the logging of mode and setpoint.
// Illustrative, not compilable; the full implementation is not public.
//
// The subsystem owns its IO and its inputs. Every loop it reads the hardware (or the simulation)
// once, and AdvantageKit logs exactly what the code then sees.

public class MotorSubsystem<T extends MotorInputsAutoLogged, U extends MotorIO>
        extends SubsystemBase {
    protected final U io;       // MotorIOTalonFX on the robot, MotorIOSim in simulation
    protected final T inputs;

    @Override
    public void periodic() {
        io.readInputs(inputs);
        Logger.processInputs(getName(), inputs);
    }

    /** Open-loop voltage command, clamped to -12..12 V. */
    public Command runVoltage(double voltage) {
        return Commands.run(() -> io.setVoltage(MathUtil.clamp(voltage, -12.0, 12.0)), this);
    }
}
