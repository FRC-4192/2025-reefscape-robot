package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.function.DoubleSupplier;

public class GroundTake extends SubsystemBase {
    private final TalonFX wrist;
    private final SparkFlex take;

    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(11, 14);
    private final ProfiledPIDController controller = new ProfiledPIDController(0.8, 0, 0, constraints);
    private final PIDController simpleController = new PIDController(.5, 0, 0);

    public static final double WRIST_RATIO = 1;

    private State state = State.HOLDING;

    private Angle offsetOffset = Units.Degrees.zero();

    public enum State {
        INTAKING(0),
        HOLDING(0);

        private final Angle angle;

        State(Angle angle) {
            this.angle = angle;
        }
        State(double degrees) {
            this(Units.Degrees.of(degrees));
        }

        public Angle angle() {
            return angle;
        }
    }

    public GroundTake() {
        wrist = new TalonFX(-1);
        take = new SparkFlex(-1, MotorType.kBrushless);

        // todo configuration for motors
    }

    public State getState() {
        return state;
    }
    public Angle getTarget() {
        return state.angle;
    }

    public boolean isSafeToLift() {
        return getState() == State.HOLDING;
    }

    public Angle getPosition() {
        return wrist.getPosition().getValue().times(WRIST_RATIO).plus(Constants.ArmConstants.START_HORIZONTAL_OFFSET).plus(offsetOffset);
    }
    public AngularVelocity getVelocity() {
        return wrist.getVelocity().getValue().times(WRIST_RATIO);
    }
    public AngularAcceleration getAcceleration() {
        return wrist.getAcceleration().getValue().times(WRIST_RATIO);
    }

    public Current getCurrent() {
        return wrist.getTorqueCurrent().getValue();
    }

    public Command setTargetStay(State state) {
        return runOnce(() -> this.state = state).andThen(runTo());//.andThen(stay());
    }
    public Command setTargetOnly(State state) {
        return runOnce(() -> this.state = state).andThen(runTo());
    }

//    public Command adjust(DoubleSupplier radiansPerSecond) {
//        MutAngle startAngle = getPosition().mutableCopy();
//        return runBasic(() -> startAngle.mut_plus(radiansPerSecond.getAsDouble() * Constants.period, Units.Radians));
//    }

    public Command rezero(DoubleSupplier power) {
        return run(() -> wrist.set(power.getAsDouble())).finallyDo(() -> offsetOffset = offsetOffset.plus(Constants.ArmConstants.START_HORIZONTAL_OFFSET.minus(getPosition())));
    }
    public Command rezero() {
        return rezero(() -> .075);
    }


    @Override
    public void periodic() {
//        SmartDashboard.putNumber("Take Current", originalCurrent);
//        SmartDashboard.putNumber("Take Current w/ Low Pass Filter", filteredCurrent);
    }


    /**
     * Returns a command that runs the arm to the target using motion profiled PID.
     */
    private Command runTo() {
        return new FunctionalCommand(
                () -> controller.reset(getPosition().in(Units.Radians), getVelocity().in(Units.RadiansPerSecond)),
                () -> wrist.set(
                        controller.calculate(getPosition().in(Units.Radians), getTarget().in(Units.Radians))
//                                + feedforward(getPosition(), controller.getSetpoint().velocity)
                ),
                (interrupted) -> wrist.set(0),
                controller::atGoal,
                this
        );
    }


}
