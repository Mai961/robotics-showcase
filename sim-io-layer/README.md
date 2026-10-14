# sim-io-layer: one motor interface, a real and a simulated implementation

See the project site: https://mai961.github.io/

## What it does

It separates the robot's control code from its hardware. Every mechanism talks to its motor
through one interface, `MotorIO`. On the robot that interface is backed by a CTRE TalonFX motor
controller (`MotorIOTalonFX`); in simulation, by a kinematic model (`MotorIOSim`). The swerve drive
modules have their own interface, with an equally simple simulated version
(`SwerveModuleIOSimpleSim`). The control code is the same on the robot and in simulation.

## Why it exists

The robot program has to run in WPILib simulation for testing, and its match logs have to show
exactly what the code saw. Putting all hardware access behind an IO interface, with one inputs
object that is read once per loop and logged, gives both. The interface's comment sets the ownership
rule (paraphrased here): the subsystem owns its IO and logs its inputs every loop. The kinematic
model was chosen on purpose; its class comment reads: "A simple kinematic motor simulation that
moves directly according to its limits. This avoids the complexity and potential bugs of a full
physics simulation when realism is not required."

## How it works

```
 subsystem (turret, shooter, ...) ──► MotorIO ──┬── MotorIOTalonFX: TalonFX runs the closed loops
        ▲                                       └── MotorIOSim:     kinematic model in the robot loop
        └── MotorInputs (read once per loop, logged by AdvantageKit)

 swerve drive ──► SwerveModuleIO ──► SwerveModuleIOSimpleSim: module is in its commanded state
```

The real or simulated implementation is picked once, when the robot program builds the
subsystem. From the source (`RobotContainer.java`):

```java
isReal
        ? new MotorIOTalonFX(TurretConfig.TURRET_CONFIG)
        : new MotorIOSim(TurretConfig.TURRET_CONFIG),
```

1. `MotorIO.java` is the contract: read inputs, and set voltage, a velocity with feedforward volts,
   or a profiled position. All in mechanism units.
2. `MotorInputs.java` holds the values read each loop. `@AutoLog` makes AdvantageKit log every
   field.
3. `MotorSubsystem.java`: `periodic` reads the IO once and logs it, and commands only call the IO.
4. `MotorIOTalonFX.java` sets the gear ratio on the controller, lowers the status-signal rate to
   100 Hz to save CAN bandwidth, and sends velocity and Motion Magic requests that the controller
   runs itself.
5. `MotorIOSim.java`: `readInputs` advances a kinematic model. Voltage maps linearly to speed, a
   velocity setpoint is reached at once (capped at the motor's free speed), and a position setpoint
   follows a trapezoid profile.
6. `SwerveModuleIOSimpleSim.java`: each simulated module is exactly in its commanded state (wheel
   speed, steering angle), and drive distance is integrated from the speed. The library also has a
   heavier variant built on WPILib's `DCMotorSim` (motor model, inertia, gear ratio), but the robot
   program wired in this simple one.

## Interfaces

| Method | Real (TalonFX) | Simulated |
|---|---|---|
| `readInputs(MotorInputs)` | reads the controller's status signals | reports the kinematic state; current is 0 |
| `setVoltage(volts)` | `VoltageOut` | speed = volts / 12 x free speed |
| `setVelVoltSetpoint(velocity, ffVolts)` | `VelocityVoltage` with feedforward, on the controller | velocity set directly; feedforward ignored |
| `setMotionMagicSetpoint(position)` | `MotionMagicVoltage`, profiled on the controller | trapezoid profile in the model |

## How it was run

The robot program's motor subsystems (turret, shooter, hood, intake and others) used `MotorIO`. On
the robot the TalonFX implementation ran. In WPILib simulation, the same program ran with
`MotorIOSim` and `SwerveModuleIOSimpleSim`, and the coprocessor stack could connect to it over the
UDP bridge.

## Replay

No recording is included for this module. The [`control/turret-2026`](../control/turret-2026/)
replay shows `MotorIOTalonFX` at work on the robot: its `motor_*` columns are the turret motor's
`MotorInputs` fields as the robot program logged them.

## Status

**Status: infrastructure excerpt.** All files except `MotorInputs.java` (a faithful excerpt) are
simplified from the source. They need WPILib, CTRE Phoenix 6 and AdvantageKit, and will not
compile on their own.

## Credits

Written by Zile Liao, with contributions from teammates: 32 of 52 non-merge commits on the excerpted
files are his (`git log --no-merges --format=%an -- <files>` in the source repository).
