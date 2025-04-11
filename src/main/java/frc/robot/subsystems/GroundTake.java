package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LowPassFilter2;
import frc.lib.util.TimedCondition;
import frc.robot.Constants.GroundConstants;
import frc.robot.Constants.IntakeConstants;

import java.util.function.DoubleSupplier;

public class GroundTake extends SubsystemBase {
    // private final TalonFX wrist;
    private final SparkFlex take;
    // private final ColorSensorV3 colorSensor;

    // private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(1, 2);
    // private final ProfiledPIDController controller = new ProfiledPIDController(0.5, 0, .01, constraints);
    // private final PIDController simpleController = new PIDController(.5, 0, 0);

    public static final double WRIST_RATIO = 1.0/25.0/4.5;
    // public static final double CORAL_CLOSE_THRESHOLD = 1477; // texas TORKKKKK

    private double adjust = 0.0;
    private State state = State.HOLDING;
    private Angle offsetOffset = Units.Degrees.zero();

    private LowPassFilter2 sensorFilter = new LowPassFilter2(.9);
    // private double closeness = 0;
    private int toggle = 0;
    
    public enum State {
        INTAKING(1),
        HOLDING(105),
        SCORING(93);

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
        // wrist = new TalonFX(17);
        take = new SparkFlex(18, MotorType.kBrushless);
        // colorSensor = new ColorSensorV3(I2C.Port.kMXP);

        // todo configuration for motors
        // wrist.getConfigurator().apply(GroundConstants.wristConfig);
        take.configure(GroundConstants.takeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // controller.setTolerance(.015);
        // controller.reset(getPosition().in(Units.Radians), getVelocity().in(Units.RadiansPerSecond));
        // simpleController.setTolerance(.01);
    }

    public State getState() {
        return state;
    }
    public Angle getTarget() {
        return state.angle;
    }

    // public Angle getPosition() {
    //     return wrist.getPosition().getValue().times(WRIST_RATIO).plus(Constants.GroundConstants.START_HORIZONTAL_OFFSET).plus(offsetOffset);
    // }
    // public AngularVelocity getVelocity() {
    //     return wrist.getVelocity().getValue().times(WRIST_RATIO);
    // }
    // public AngularAcceleration getAcceleration() {
    //     return wrist.getAcceleration().getValue().times(WRIST_RATIO);
    // }

    // public Current getWristCurrent() {
    //     return wrist.getTorqueCurrent().getValue();
    // }
    public Current getTakeCurrent() {
        return Units.Amps.of(take.getOutputCurrent());
    }

    // public boolean hasCoral() {
    //     return closeness > CORAL_CLOSE_THRESHOLD;
    // }

    

    // public Command setTargetStay(State state) {
    //     return runOnce(() -> this.state = state).andThen(runTo()).andThen(stay());
    // }
    // public Command setTargetOnly(State state) {
    //     return runOnce(() -> this.state = state).andThen(runTo());
    // }


    @Override
    public void periodic() {
        // closeness = sensorFilter.update(colorSensor.getProximity());
        // SmartDashboard.putNumber("Ground Coral Closeness", closeness);

        SmartDashboard.putNumber("GroundTake Current", getTakeCurrent().in(Units.Amps));

        // SmartDashboard.putNumber("GroundWrist Current (A)", getWristCurrent().in(Units.Amps));
        // SmartDashboard.putNumber("GroundWrist Power", wrist.get());
        // SmartDashboard.putNumber("GroundWrist Position", getPosition().in(Units.Degree));
//        SmartDashboard.putNumber("GroundWrist Velo", getVelocity().in(Units.DegreesPerSecond));
//        SmartDashboard.putNumber("GroundWrist Accel", getAcceleration().in(Units.DegreesPerSecondPerSecond));

        // SmartDashboard.putNumberArray("GroundWrist Pos(array)", new double[] { Math.toDegrees(controller.getSetpoint().position), getPosition().in(Units.Degrees) });
        // SmartDashboard.putNumberArray("GroundWrist Vel(array)", new double[] { Math.toDegrees(controller.getSetpoint().velocity), getVelocity().in(Units.DegreesPerSecond) });
    }


//    public Command runWristRaw(DoubleSupplier power) {
//        return run(() -> wrist.set(power.getAsDouble()));
//    }
//    public Command runWristRawFeedforward(DoubleSupplier power) {
//        return run(() -> wrist.set(power.getAsDouble() + feedforward(getPosition(), 0)));
//    }
//
//    public Command runWristBasic(Supplier<Angle> target) {
//        return run(() -> wrist.set(simpleController.calculate(getPosition().in(Units.Radians), target.get().in(Units.Radians)) + feedforward(getPosition(), 0)));
//    }

    // public Command stay() {
    //     return run(() -> wrist.set(feedforward(getPosition(), 0)));
    // }
    // public Command stayPID() {
    //     return run(() -> wrist.set(simpleController.calculate(getPosition().in(Units.Radians), getTarget().in(Units.Radians) + adjust) + feedforward(getPosition(), 0)));
    // }


    // public static double feedforward(Angle position, double velocity) {
    //     return GroundConstants.feedforward.calculate(position.in(Units.Radians), velocity) / RobotController.getBatteryVoltage();
    // }

    // public Command rezero(DoubleSupplier power) {
    //     return run(() -> wrist.set(power.getAsDouble())).finallyDo(() -> offsetOffset = offsetOffset.plus(Constants.GroundConstants.START_HORIZONTAL_OFFSET.minus(getPosition())));
    // }
    // public Command rezero() {
    //     return rezero(() -> .075);
    // }

    /**
     * Returns a command that runs the arm to the target using motion profiled PID.
     */
    // private Command runTo() {
    //     return new FunctionalCommand(
    //         // () -> controller.reset(getPosition().in(Units.Radians), getVelocity().in(Units.RadiansPerSecond)),
    //         () -> controller.reset(getVelocity().in(Units.RadiansPerSecond) < .01 ? new TrapezoidProfile.State(getPosition().in(Units.Radians), getVelocity().in(Units.RadiansPerSecond)) : controller.getSetpoint()),
    //         () -> wrist.set(
    //             controller.calculate(getPosition().in(Units.Radians), getTarget().in(Units.Radians) + adjust)
    //                 + feedforward(getPosition(), controller.getSetpoint().velocity)
    //         ),
    //         (interrupted) -> wrist.set(interrupted ? wrist.get() : feedforward(getPosition(), 0)),
    //         controller::atGoal,
    //         this
    //     );
    // }


    // public Command outtakeCoral(DoubleSupplier power) {
    //     return run(() -> runTakeRaw(-power.getAsDouble()));
    // }
    // public Command intakeCoral(DoubleSupplier power) {
    //     return run(() -> runTakeRaw(power.getAsDouble())).until(this::hasCoral);
    // }

    // public Command intakeCoralSequence(DoubleSupplier power) {
    //     return intakeCoral(power).andThen(setTargetOnly(State.SCORING).raceWith(Commands.run(() -> runTakeRaw(power.getAsDouble()))));
    // }
    // public Command toggleState(int x){
    //     toggle= (toggle + x) % 3;
    //     switch (toggle) {
    //         case 0:
    //             return setTargetOnly(State.INTAKING);
    //         case 1:
    //             return setTargetOnly(State.INTAKING).andThen(intakeCoralSequence(() -> 1));
    //         case 2:
    //             return outtakeCoral( () -> 1);
    //         default:
    //             return setTargetOnly(State.INTAKING);
    //     }
    // }
    

    public Command runIntake(DoubleSupplier power) {
        return run(() -> runTakeRaw(power.getAsDouble()));
    }
    public Command runOuttake(DoubleSupplier power) {
        return run(() -> runTakeRaw(-power.getAsDouble()));
    }

    public void runTakeRaw(double power) {
        take.set(power);
    }

    public Command runTakeOnce(double power) {
        return runOnce(() -> runTakeRaw(power));
    }


}
