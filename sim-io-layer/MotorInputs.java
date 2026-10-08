// Faithful excerpt of the robot library's io/MotorInputs.java, unchanged except for this header
// and the removed package line. @AutoLog makes AdvantageKit generate MotorInputsAutoLogged, which
// records every field to the match log each loop.

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class MotorInputs {
    public double velocityRotPerSecond = 0.0;
    public double positionRot = 0.0;
    public double appliedVolts = 0.0;
    public double motorVolts = 0.0;
    public double currentStatorAmps = 0.0;
    public double currentSupplyAmps = 0.0;
}
