package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.robot.Constants.ElevatorConstants;

@SuppressWarnings("unused")
public class Elevator extends SubsystemBase {
    private final TalonFX motor, motor2;

    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(2, 2.5);
    private final ProfiledPIDController controller = new ProfiledPIDController(2.0, 0, 0.01, constraints, Constants.period);
    private final PIDController basicController = new PIDController(1.0, 0, 0);

    private State state = State.L0;

    private MutDistance offsetOffset = Units.Meters.zero().mutableCopy();

    public enum State {
        ALGAELOW(0.26162),
        ALGAEHIGH(0.6925),
        L0(0.00),
        L1(0.244),
        L2(0.0),//.167
        L3(0.153), //.24 
        L4(0.74); //.75

        private final Distance position;

        State(Distance position) {
            this.position = position;
        }
        State(double meters) {
            this(Units.Meters.of(meters));
        }

        public double meters() {
            return position.baseUnitMagnitude();
        }
    }

    public Elevator() {
        motor = new TalonFX(14);
        motor2 = new TalonFX(15);

        motor.getConfigurator().apply(ElevatorConstants.elevatorConfig);
        motor2.getConfigurator().apply(ElevatorConstants.elevatorConfig);
        motor2.setControl(new Follower(motor.getDeviceID(), true));

        controller.setTolerance(0.01, .01);
        controller.reset(getPosition().in(Units.Meters), getVelocity().in(Units.MetersPerSecond));
        basicController.setTolerance(0.02);

        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                voltage -> motor.setVoltage(voltage.in(Units.Volts)),
                log -> log.motor("elevator")
                    .voltage(motor.getMotorVoltage().getValue())
                    .linearPosition(getPosition())
                    .linearVelocity(getVelocity())
                    .current(getCurrent()),
                this
            )
        );

        rezero();
        setDefaultCommand(stay());
    }

    public State getState() {
        return state;
    }

    public boolean isSafeToIntake() {
        return getState() == State.L0 || getState() == State.L1 || getState() == State.L3;
    }

    public Distance getPosition() {
        return motorToHeight((motor.getPosition().getValueAsDouble() + motor2.getPosition().getValueAsDouble()) * .5).plus(offsetOffset);
    }
    public LinearVelocity getVelocity() {
        return motorToHeight((motor.getVelocity().getValueAsDouble() + motor2.getVelocity().getValueAsDouble()) * .5).div(Units.Second.one());
    }

    public Current getCurrent() {
        return Units.Amps.of(motor.getStatorCurrent().getValueAsDouble() + motor2.getStatorCurrent().getValueAsDouble());
    }

    public Command setTargetStay(State state) {
        return runOnce(() -> this.state = state).andThen(runTo()).andThen(stay());
    }
    public Command setTargetOnly(State state) {
        return runOnce(() -> this.state = state).andThen(runTo());
    }

    public Command adjust(DoubleSupplier metersPerSecond) {
        MutDistance startPosition = getPosition().mutableCopy();
        return runBasic(() -> startPosition.mut_plus(metersPerSecond.getAsDouble() * Constants.period, Units.Meters));
    }

    public void rezero() {
        offsetOffset.mut_minus(getPosition());
    }
    public Command rezeroCommand() {
        return runOnce(this::rezero);
    }

    public Distance getError() {
        return getState().position.minus(getPosition());
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Target", getState().meters());
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
        return new FunctionalCommand(
            () -> controller.reset(controller.getSetpoint()),
            () -> motor.set(
                controller.calculate(getPosition().in(Units.Meters), getState().meters())
                    + feedforward(controller.getSetpoint().velocity)
            ),
            (interrupted) -> motor.set(interrupted ? 0 : ElevatorConstants.feedforward.calculate(0)),
            controller::atGoal,
            this
        ).withName("run to " + getState().toString());
    }

    @Deprecated
    public void runBasic() {
        motor.set((basicController.atSetpoint() ? 0 : basicController.calculate(getPosition().in(Units.Meters), getState().meters()))
             + feedforward(getState().meters() - getPosition().in(Units.Meters))
        );
    }

    public Command runRaw(DoubleSupplier power) {
        return run(() -> motor.set(power.getAsDouble()));
    }
    public Command runRawFeedforward(DoubleSupplier power) {
        return run(() -> motor.set(power.getAsDouble() + feedforward(0)));
    }

    public Command runBasic(Supplier<Distance> target) {
        return run(() -> motor.set(basicController.calculate(getPosition().in(Units.Meters), target.get().in(Units.Meters)) + feedforward(0)));
    }

    public Command stay() {
        return run(() -> motor.set(getPosition().in(Units.Meters) < .03 ? 0 : feedforward(0))).withName("stay");
    }
    public Command stayPID() {
        return run(() -> motor.set(getPosition().in(Units.Meters) < .05 ? 0 : (basicController.calculate(getPosition().in(Units.Meters), getState().meters()) + feedforward(0)))).withName("stay");
    }


    public static Distance motorToHeight(double rotations) {
        return Units.Inches.of(rotations * 2. * Math.PI / 3. * .8755);
    }

    public static double feedforward(double velocity) {
        return ElevatorConstants.feedforward.calculate(velocity) / RobotController.getBatteryVoltage();
    }



    //--------------------------------------------------------------------------
    // Sys ID Stuff
    private final SysIdRoutine sysIdRoutine;

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

}

