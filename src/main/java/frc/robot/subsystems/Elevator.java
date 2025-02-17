package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase{
    private final SparkFlex motor, motor2;

    private TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(getPosition(), getCurrent());
    private ProfiledPIDController controller = new ProfiledPIDController(getTarget(), getPosition(), getCurrent(), constraints);

    private double target;
    private State state = State.L0;

    public enum State {
        L0(0),
        L1(-1),
        L2(-1),
        L3(-1),
        L4(-1);

        private final double position; // units = rotations

        private State(double position) {
            this.position = position;
        }
    }

    public Elevator(){
        motor = new SparkFlex(0, MotorType.kBrushless); // change id
        motor2 = new SparkFlex(0, MotorType.kBrushless); // change id
        SparkFlexConfig config = new SparkFlexConfig();
        config
            .idleMode(null)
            .inverted(false);
        config.softLimit
            .forwardSoftLimitEnabled(false)
            .reverseSoftLimitEnabled(false)
            .forwardSoftLimit(0)
            .reverseSoftLimit(0);
        // config.closedLoop.pidf(target, target, target, target)
        motor.configure(config, null, null);
        motor2.configure(config.follow(motor), null, null);
    }

    private void motorSetup() {
        // motor.getConfigurator().apply(
        //     new TalonFXConfiguration()
        //         .withCurrentLimits(
        //             new CurrentLimitsConfigs()
        //                 .withStatorCurrentLimit(30)
        //                 .withStatorCurrentLimitEnable(true)
        //         )
        // );
        // motor.setNeutralMode(NeutralModeValue.Brake);
        // // TODO: torque control or percent supply output control?
        // motor2.setControl(new Follower(9, false));
        // motor3.setControl(new Follower(9, false));
        // motor2.setNeutralMode(NeutralModeValue.Brake);
        // motor3.setNeutralMode(NeutralModeValue.Brake);
        // motor2.getConfigurator().apply(
        //     new TalonFXConfiguration()
        //         .withCurrentLimits(
        //             new CurrentLimitsConfigs()
        //                 .withStatorCurrentLimit(30)
        //                 .withStatorCurrentLimitEnable(true)
        //         )
        // );
        // motor3.getConfigurator().apply(
        //     new TalonFXConfiguration()
        //         .withCurrentLimits(
        //             new CurrentLimitsConfigs()
        //                 .withStatorCurrentLimit(30)
        //                 .withStatorCurrentLimitEnable(true)
        //         )
        // );
    }

    public double getTarget() {
        return state.position;
    }
    public double getPosition() {
        return 0;
    }
    public double getVelocity() {
        return 0;
    }

    public double getCurrent() {
        return motor.getOutputCurrent() + motor2.getOutputCurrent();
    }

    public void setTarget(State state) {
        this.state = state;
        runTo();
    }

    @Override
    public void periodic() {
        // motor.set(ElevatorConstants.pidConroller.calculate(getPosition(), getTarget()) + ElevatorConstants.feedforward.calculate())
        SmartDashboard.putNumber("Elevator Target", getTarget());
        SmartDashboard.putNumber("Elevator Position", getPosition());
        SmartDashboard.putNumber("Elevator Current (A)", getCurrent());
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
            () -> controller.reset(getPosition(), getVelocity()),
            () -> motor.set(
                controller.calculate(getPosition(), getTarget())
                    + ElevatorConstants.feedforward.calculate(controller.getSetpoint().velocity) / RobotController.getBatteryVoltage()
            ),
            (interrupted) -> motor.set(0),
            controller::atGoal,
            this
        );
    }

}

