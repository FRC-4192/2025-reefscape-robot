package frc.robot.subsystems;

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

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.robot.Constants.ArmConstants;

@SuppressWarnings("unused")
public class Arm extends SubsystemBase {
    private final TalonFX motor;

    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(8, 15);
    private final ProfiledPIDController controller = new ProfiledPIDController(1.6, 0, .01, constraints);
    private final PIDController simpleController = new PIDController(.5, 0, 0);

    public static final double WRIST_RATIO = 1./16/4.5;

    private State state = State.HOLDING;
    private double adjust = 0.0;

    private Angle offsetOffset = Units.Degrees.zero();

    public enum State {
        INTAKING(-72),
        HOLDING(110),
        DUNKING(90 - 32.211),
        SCORING(99),
        EJECTING(-80), // L1 scoring or throwing away unwanted coral
        ALGAE(-55);

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
    
    public Arm() {
        motor = new TalonFX(9);
        motor.getConfigurator().apply(ArmConstants.wristConfig);

        controller.setTolerance(.015);
        controller.reset(getPosition().in(Units.Radians), getVelocity().in(Units.RadiansPerSecond));
        simpleController.setTolerance(.01);

        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                voltage -> motor.setVoltage(voltage.in(Units.Volts)),
                log -> log.motor("wrist")
                    .voltage(motor.getMotorVoltage().getValue())
                    .angularPosition(getPosition())
                    .angularVelocity(getVelocity())
                    .angularAcceleration(getAcceleration())
                    .current(getCurrent()),
                this
            )
        );

        setDefaultCommand(stay());
    }

    public State getState() {
        return state;
    }
    public Angle getTarget() {
        return state.angle;
    }

    public boolean isSafeToLift() {
        return getState() == State.HOLDING || getState() == State.SCORING;
    }

    public Angle getPosition() {
        return motor.getPosition().getValue().times(WRIST_RATIO).plus(ArmConstants.START_HORIZONTAL_OFFSET).plus(offsetOffset);
    }
    public AngularVelocity getVelocity() {
        return motor.getVelocity().getValue().times(WRIST_RATIO);
    }
    public AngularAcceleration getAcceleration() {
        return motor.getAcceleration().getValue().times(WRIST_RATIO);
    }

    public Current getCurrent() {
        return motor.getTorqueCurrent().getValue();
    }

    public Command setTargetStay(State state) {
        adjust = 0;
        return runOnce(() -> this.state = state).andThen(runTo()).andThen(stay());        
    }
    public Command setTargetOnly(State state) {
        adjust = 0;
        return runOnce(() -> this.state = state).andThen(runTo());        
    }

    public Command adjust(DoubleSupplier radiansPerSecond) {
        MutAngle startAngle = getPosition().mutableCopy();
        return runBasic(() -> startAngle.mut_plus(radiansPerSecond.getAsDouble() * Constants.period, Units.Radians));
    }

    public Command rezero(DoubleSupplier power) {
        return runEnd(() -> motor.set(power.getAsDouble()), stay()::execute).finallyDo(() -> offsetOffset = offsetOffset.plus(ArmConstants.START_HORIZONTAL_OFFSET.minus(getPosition())));
    }
    public Command rezero() {
        return rezero(() -> .075);
    }

    public Angle getError() {
        return getState().angle.minus(getPosition());
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("Wrist Current (A)", getCurrent().in(Units.Amp));
        SmartDashboard.putNumber("Wrist Power", motor.get());
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
            // () -> controller.reset(getPosition().in(Units.Radians), getVelocity().in(Units.RadiansPerSecond)),
            () -> controller.reset(getVelocity().in(Units.RadiansPerSecond) < .01 ? new TrapezoidProfile.State(getPosition().in(Units.Radians), getVelocity().in(Units.RadiansPerSecond)) : controller.getSetpoint()),
            () -> motor.set(
                controller.calculate(getPosition().in(Units.Radians), getTarget().in(Units.Radians) + adjust)
                    + feedforward(getPosition(), controller.getSetpoint().velocity)
            ),
            (interrupted) -> motor.set(interrupted ? motor.get() : feedforward(getPosition(), 0)),
            controller::atGoal,
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
        return run(() -> motor.set(simpleController.calculate(getPosition().in(Units.Radians), getTarget().in(Units.Radians) + adjust) + feedforward(getPosition(), 0)));
    }


    public static double feedforward(Angle position, double velocity) {
        return ArmConstants.feedforward.calculate(position.in(Units.Radians), velocity) / RobotController.getBatteryVoltage();
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
