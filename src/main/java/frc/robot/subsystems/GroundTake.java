package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
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
import frc.lib.util.ExponentialLowPassFilter;
import frc.robot.Constants;
import frc.robot.Constants.GroundConstants;
import frc.robot.Constants.IntakeConstants;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class GroundTake extends SubsystemBase {
    private final TalonFX wrist;
    private final SparkFlex take;

    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(11, 14);
    private final ProfiledPIDController controller = new ProfiledPIDController(0.8, 0, 0, constraints);
    private final PIDController simpleController = new PIDController(.5, 0, 0);


    public static final double WRIST_RATIO = 1/5/5;

    private double adjust = 0.0;

    private State state = State.HOLDING;

    private Angle offsetOffset = Units.Degrees.zero();

    private double originalCurrent;
    private double filteredCurrent;

    private ExponentialLowPassFilter filter = new ExponentialLowPassFilter(0.008, 1.7); // 0.00012, 2.8
    private boolean spike;

    public enum State {
        INTAKING(0),
        HOLDING(0),
        SCORING(0);

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
        wrist = new TalonFX(17);
        take = new SparkFlex(18, MotorType.kBrushless);

        // todo configuration for motors
        wrist.getConfigurator().apply(Constants.GroundConstants.wristConfig);
        take.configure(IntakeConstants.motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        controller.setTolerance(.015);
        controller.reset(getPosition().in(Units.Radians), getVelocity().in(Units.RadiansPerSecond));
        simpleController.setTolerance(.01);


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
        return wrist.getPosition().getValue().times(WRIST_RATIO).plus(Constants.GroundConstants.START_HORIZONTAL_OFFSET).plus(offsetOffset);
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
        adjust=0;
        return runOnce(() -> this.state = state).andThen(runTo());//.andThen(stay());
    }
    public Command setTargetOnly(State state) {
        adjust=0;
        return runOnce(() -> this.state = state).andThen(runTo());
    }

    

//    public Command adjust(DoubleSupplier radiansPerSecond) {
//        MutAngle startAngle = getPosition().mutableCopy();
//        return runWristBasic(() -> startAngle.mut_plus(radiansPerSecond.getAsDouble() * Constants.period, Units.Radians));
//    }

    public Command runWristRaw(DoubleSupplier power) {
        return run(() -> wrist.set(power.getAsDouble()));
    }
    public Command runWristRawFeedforward(DoubleSupplier power) {
        return run(() -> wrist.set(power.getAsDouble() + feedforward(getPosition(), 0)));
    }

    public Command runWristBasic(Supplier<Angle> target) {
        return run(() -> wrist.set(simpleController.calculate(getPosition().in(Units.Radians), target.get().in(Units.Radians)) + feedforward(getPosition(), 0)));
    }

    public Command stay() {
        return run(() -> wrist.set(feedforward(getPosition(), 0)));
    }
    public Command stayPID() {
        return run(() -> wrist.set(simpleController.calculate(getPosition().in(Units.Radians), getTarget().in(Units.Radians) + adjust) + feedforward(getPosition(), 0)));
    }


    public static double feedforward(Angle position, double velocity) {
        return GroundConstants.feedforward.calculate(position.in(Units.Radians), velocity) / RobotController.getBatteryVoltage();
    }

    public Command rezero(DoubleSupplier power) {
        return run(() -> wrist.set(power.getAsDouble())).finallyDo(() -> offsetOffset = offsetOffset.plus(Constants.GroundConstants.START_HORIZONTAL_OFFSET.minus(getPosition())));
    }
    public Command rezero() {
        return rezero(() -> .075);
    }


    @Override
    public void periodic() {
        originalCurrent = take.getOutputCurrent();
        filteredCurrent = filter.update(originalCurrent);

        SmartDashboard.putNumber("Take Current", originalCurrent);
        SmartDashboard.putNumber("Take Current w/ Low Pass Filter", filteredCurrent);
        SmartDashboard.putNumber("Wrist Current (A)", getCurrent().in(Units.Amp));
        SmartDashboard.putNumber("Wrist Power", wrist.get());
        SmartDashboard.putNumber("Wrist Position", getPosition().in(Units.Degree));
        SmartDashboard.putNumber("Wrist Velo", getVelocity().in(Units.DegreesPerSecond));
        SmartDashboard.putNumber("Wrist Accel", getAcceleration().in(Units.DegreesPerSecondPerSecond));

        SmartDashboard.putNumberArray("Wrist Pos(array)", new double[] { Math.toDegrees(controller.getSetpoint().position), getPosition().in(Units.Degrees) });
        SmartDashboard.putNumberArray("Wrist Vel(array)", new double[] { Math.toDegrees(controller.getSetpoint().velocity), getVelocity().in(Units.DegreesPerSecond) });

    }


    /**
     * Returns a command that runs the arm to the target using motion profiled PID.
     */
    private Command runTo() {
        return new FunctionalCommand(
                () -> controller.reset(getPosition().in(Units.Radians), getVelocity().in(Units.RadiansPerSecond)),
                () -> wrist.set(
                        controller.calculate(getPosition().in(Units.Radians), getTarget().in(Units.Radians)+adjust)
                               + feedforward(getPosition(), controller.getSetpoint().velocity)
                ),
                (interrupted) -> wrist.set(0),
                controller::atGoal,
                this
        );
    }


    //intake stuff below
    public Command outtakeCoral(double power){
        spike=false;
        return new FunctionalCommand(
            () -> {},
            () -> {
                runTakeRaw(-power);
                spike = spike || filteredCurrent >= 50;
            },
            (x) -> runTakeRaw(0),
            () -> filteredCurrent <= 45 && spike,
            this
        );
    }
    public Command intakeCoral(DoubleSupplier power) {
        return new FunctionalCommand(
            () -> {},
            () -> {
                runTakeRaw(power.getAsDouble());
            },
            (x) -> runTakeRaw(x ? power.getAsDouble() : 0),
            () -> filteredCurrent >= 45,
            this
        );
    }

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
