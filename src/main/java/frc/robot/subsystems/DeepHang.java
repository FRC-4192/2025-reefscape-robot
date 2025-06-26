package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
// import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HangConstants;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;;

public class DeepHang extends SubsystemBase {
    private final SparkFlex motor, motor2;

    // private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(1, 2);
    // private final ProfiledPIDController controller = new ProfiledPIDController(1, 0, .01, constraints);
    private final BangBangController bangBang = new BangBangController(.01);
    private final PIDController simpleController = new PIDController(1, 0, 0);

    public static final double WRIST_RATIO = 1.0/25.0*(12.0/72.0);

    private State state = State.AIMING;

    private Angle offsetOffset = Units.Degrees.zero();

    public enum State {
        // STORING(130),/
        AIMING(83);
        // HANGING(134);

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
    
    public DeepHang() {
        motor = new SparkFlex(17, MotorType.kBrushless);
        motor2 = new SparkFlex(18, MotorType.kBrushless);

        motor.configure(HangConstants.hangConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        motor2.configure(HangConstants.hangConfig.follow(motor,true), ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        // controller.setTolerance(.015);
        // controller.reset(getPosition().in(Units.Radians), getVelocity().in(Units.RadiansPerSecond));
        simpleController.setTolerance(.01);

        // rezero();
        setDefaultCommand(stay());
    }

    public State getState() {
        return state;
    }
    public Angle getTarget() {
        return state.angle;
    }

    public Angle getPosition() {
        return Units.Rotations.of(motor.getEncoder().getPosition()).times(WRIST_RATIO).plus(HangConstants.START_HORIZONTAL_OFFSET);
    }
    public AngularVelocity getVelocity() {
        return Units.RPM.of(motor.getEncoder().getVelocity()).times(WRIST_RATIO);
    }

    public double getCurrent() {
        return motor.getOutputCurrent() + motor2.getOutputCurrent();
    }

    public double getMotorCurrent(int x){
        return (x==1) ?  motor.getOutputCurrent() : motor2.getOutputCurrent();
    }

    public Command setTargetStay(State state) {
        return runOnce(() -> this.state = state).andThen(runTo()).andThen(stay());        
    }
    public Command setTargetOnly(State state) {
        return runOnce(() -> this.state = state).andThen(runTo());        
    }
    // public Command toggleState() {
    //     State choose = state != State.AIMING ? State.AIMING : State.HANGING;
    //     return runOnce(() -> this.state = choose).andThen(runTo());
    // }

    // public Command adjust(DoubleSupplier radiansPerSecond) {
    //     MutAngle startAngle = getPosition().mutableCopy();
    //     return runBasic(() -> startAngle.mut_plus(radiansPerSecond.getAsDouble() * Constants.period, Units.Radians));
    // }

    public void rezero() {
        offsetOffset = offsetOffset.plus(HangConstants.START_HORIZONTAL_OFFSET.minus(getPosition()));
    }
    public Command rezero(DoubleSupplier power) {
        return runEnd(() -> motor.set(power.getAsDouble()), stay()::execute).finallyDo(() -> rezero());
    }

    public Angle getError() {
        return getState().angle.minus(getPosition());
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("DH Current m1", getMotorCurrent(1));
        SmartDashboard.putNumber("DH Current m2", getMotorCurrent(2));
        SmartDashboard.putNumber("DH Current", getCurrent());
        SmartDashboard.putNumber("DH Power", motor.get());
        SmartDashboard.putNumber("DH Position", getPosition().in(Units.Degrees));
        SmartDashboard.putNumber("DH Velo", getVelocity().in(DegreesPerSecond));

        SmartDashboard.putNumberArray("DH Pos(array)", new double[] { Math.toDegrees(simpleController.getSetpoint()), getPosition().in(Units.Degrees) });
        SmartDashboard.putNumberArray("DH Vel(array)", new double[] { Math.toDegrees(simpleController.getSetpoint()), getVelocity().in(RadiansPerSecond) });
    }


    /**
    * Returns a command that runs the arm to the target using motion profiled PID.
    */
    private Command runTo() {

        return new FunctionalCommand(
            () -> simpleController.reset(),
            () -> motor.set(
                simpleController.calculate(getPosition().in(Units.Radians), getTarget().in(Units.Radians))
                    + feedforward(getPosition(), getVelocity().in(RadiansPerSecond)))
               
            ,
            (interrupted) -> motor.set(interrupted ? motor.get() : feedforward(getPosition(), 0)),
            simpleController::atSetpoint,
            this
        );
    }

    public Command runRaw(DoubleSupplier power) {
        return run(() -> motor.set(power.getAsDouble()));
    }
    public Command runRawFeedforward(DoubleSupplier power) {
        return run(() -> motor.set(power.getAsDouble() + feedforward(getPosition(), 0)));
    }

    public Command runBasic(Supplier<Angle> target) {
        return run(() -> motor.set(simpleController.calculate(getPosition().in(Units.Radians), target.get().in(Units.Radians)) + feedforward(getPosition(), 0)));
    }

    public Command stay() {
        return run(() -> motor.set(feedforward(getPosition(), 0)));
    }
    public Command stayPID() {
        return run(() -> motor.set(simpleController.calculate(getPosition().in(Units.Radians), getTarget().in(Units.Radians)) + feedforward(getPosition(), 0)));
    }

    public static double feedforward(Angle position, double velocity) {
        return HangConstants.feedforward.calculate(position.in(Units.Radians), velocity) / RobotController.getBatteryVoltage();
    }
}
