package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static frc.robot.Constants.ElevatorConstants;

import java.util.function.DoubleSupplier;

public class Elevator extends SubsystemBase {
    private final TalonFX motor, motor2;

    private TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(2.5, 4);
    private ProfiledPIDController controller = new ProfiledPIDController(2.5, 0, 0.0001, constraints);
    private PIDController basicController = new PIDController(0.1, 0, 0);

    // private double target;
    private State state = State.L0;

    public enum State {
        L0(0),
        L1(0.02),
        L2(0.5),
        L3(0.22),
        L4(0.75);

        private final Distance position; // units = motor rotations

        private State(Distance position) {
            this.position = position;
        }
        private State(double meters) {
            this(Units.Meters.of(meters));
        }

        public double meters() {
            return position.baseUnitMagnitude();
        }
        // public double inches() {
        //     return motorToHeight(position).in(Units.Inches);
        // }
        // public double radians() {
        //     return position * 2 * Math.PI;
        // }
    }

    public Elevator() {
        motor = new TalonFX(14);
        motor2 = new TalonFX(15);
        // SparkFlexConfig config = new SparkFlexConfig();
        // config
        //     .idleMode(IdleMode.kCoast)
        //     .inverted(true)
        //     .smartCurrentLimit(30, 25);
        // config.softLimit
        //     .forwardSoftLimitEnabled(false)
        //     .reverseSoftLimitEnabled(false)
        //     .forwardSoftLimit(0)
        //     .reverseSoftLimit(0);
        // config.closedLoop.pidf(target, target, target, target)

        // motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // motor2.configure(config.follow(motor, true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        motor.getConfigurator().apply(ElevatorConstants.elevatorConfig);
        motor2.getConfigurator().apply(ElevatorConstants.elevatorConfig);
        motor2.setControl(new Follower(motor.getDeviceID(), true));
        // motor2.configure(config.inverted(true), ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        controller.setTolerance(0.011, .01);
        basicController.setTolerance(0.02);

        // routine = new SysIdRoutine(
        //     new SysIdRoutine.Config(),
        //     new SysIdRoutine.Mechanism(
        //         voltage -> {
        //             motor.setVoltage(voltage.in(Units.Volts));
        //         },
        //         log -> {
        //             log.motor("elevator")
        //                 .voltage(Units.Volts.of(motor.getBusVoltage()))
        //                 .linearPosition(getPosition())
        //                 .linearVelocity(getVelocity())
        //                 .current(getCurrent());
        //         },
        //         this
        //     )
        // );

        routine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                voltage -> {
                    motor.setVoltage(voltage.in(Units.Volts));
                },
                log -> {
                    log.motor("wrist")
                        .voltage(motor.getMotorVoltage().getValue())
                        .linearPosition(getPosition())
                        .linearVelocity(getVelocity())
                        .current(getCurrent());
                },
                this
            )
        );

        setDefaultCommand(stay());
    }


    public State getTarget() {
        return state;
    }
    // public double getPosition() {
    //     return motor.getEncoder().getPosition();
    // }
    // public double getVelocity() {
    //     return 0;
    // }
    public double getError() {
        return getTarget().meters()-getPosition().in(Units.Meters);
    }
    public double getError(double target) {
        return target-getPosition().in(Units.Meters);
    }
    public Distance getPosition() {
        return motorToHeight((motor.getPosition().getValueAsDouble() + motor2.getPosition().getValueAsDouble()) * .5);
    }
    public LinearVelocity getVelocity() {
        return motorToHeight((motor.getVelocity().getValueAsDouble() + motor2.getVelocity().getValueAsDouble()) * .5).div(Units.Second.one());
    }

    public Current getCurrent() {
        return Units.Amps.of(motor.getStatorCurrent().getValueAsDouble() + motor2.getStatorCurrent().getValueAsDouble());
    }

    public Command setTarget(State state) {
        // basicController.setSetpoint(state.meters());
        return runOnce(() -> this.state = state).andThen(runTo()).andThen(stay());
    }
    public Command setTarget2(State state) {
        // basicController.setSetpoint(state.meters());
        return runOnce(() -> this.state = state).andThen(runTo());
    }

    public boolean safeToIntake() {
        return getTarget() == State.L0;
    }

    @Override
    public void periodic() {
        // motor.set(ElevatorConstants.pidConroller.calculate(getPosition(), getTarget()) + ElevatorConstants.feedforward.calculate())
        SmartDashboard.putNumber("Elevator Target", getTarget().meters());
        SmartDashboard.putNumber("Elevator Position", getPosition().in(Units.Meters));
        // SmartDashboard.putNumber("Elevator 15", motor2.getEncoder().getPosition());
        SmartDashboard.putNumber("Elevator Current (A)", getCurrent().in(Units.Amps));
        SmartDashboard.putNumber("Elevator Power", motor.get());
        // SmartDashboard.putNumber("Elevator Set Pos", controller.getSetpoint().position);
        // SmartDashboard.putNumber("Elevator Set Velo", controller.getSetpoint().velocity);
        SmartDashboard.putNumberArray("Elevator Pos", new double[] { controller.getSetpoint().position, getPosition().in(Units.Meters) });
        SmartDashboard.putNumberArray("Elevator Vel", new double[] { controller.getSetpoint().velocity, getVelocity().in(Units.MetersPerSecond) });

        SmartDashboard.putString("Elevator Command", getCurrentCommand() == null ? "null" : getCurrentCommand().getName());
    }

    /**
    * Returns a command that runs the elevator to the target using motion profiled PID.
    */
    private Command runTo() {
        // return startRun(
        //     () -> controller.reset(getPosition(), getVelocity()),
        //     () -> motor.set(
        //         controller.calculate(getPosition(), getTarget())
        //             // Divide feedforward voltage by battery voltage to normalize it to [-1, 1]
        //             + ElevatorConstants.feedforward.calculate(controller.getSetpoint().velocity) / RobotController.getBatteryVoltage()))
        // .until(controller::atGoal)
        // .finallyDo(() -> motor.set(0));

        return new FunctionalCommand(
            () -> controller.reset(getPosition().in(Units.Meters), getVelocity().in(Units.MetersPerSecond)),
            () -> motor.set(
                controller.calculate(getPosition().in(Units.Meters), getTarget().meters())
                    + feedforward(controller.getSetpoint().velocity)
            ),
            (interrupted) -> motor.set(interrupted ? 0 : ElevatorConstants.feedforward.calculate(0)),
            controller::atGoal,
            this
        ).withName("run to " + getTarget().toString());
    }

    public void runRaw(double power) {
        motor.set(power);
    }

    @Deprecated
    public void runBasic() {
        motor.set((basicController.atSetpoint() ? 0 : basicController.calculate(getPosition().in(Units.Meters), getTarget().meters()))
             + feedforward(getTarget().meters() - getPosition().in(Units.Meters))
        );
    }

    public Command runRaw(DoubleSupplier power) {
        return run(() -> motor.set(power.getAsDouble()));
    }
    public Command runRawg(DoubleSupplier power) {
        return run(() -> motor.set(power.getAsDouble() + feedforward(power.getAsDouble())));
    }

    // public Command runBasic(DoubleSupplier )

    public Command stay() {
        return run(() -> motor.set(getPosition().in(Units.Meters) < .05 ? 0 : feedforward(0))).withName("Stay");
    }


    public static Distance motorToHeight(double rotations) {
        return Units.Inches.of(rotations * 2. * Math.PI / 3. * .8755);
    }

    public static double feedforward(double velocity) {
        return ElevatorConstants.feedforward.calculate(velocity) / RobotController.getBatteryVoltage();
    }



    //--------------------------------------------------------------------------
    // Sys ID Stuff
    private final SysIdRoutine routine;

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }
}

