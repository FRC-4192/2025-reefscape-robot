package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
    private TalonFX wrist;
    private SparkFlex take;

    private TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(0, 0);
    private ProfiledPIDController controller = new ProfiledPIDController(0, 0, 0, constraints);

    public static final double WRIST_RATIO = 1./16/4.5;

    private State state = State.HOLDING;

    public enum State {
        INTAKING(-80),
        HOLDING(105),
        SCORING(90 - 32.211),
        EJECTING(-80), // L1 scoring or throwing away unwanted coral
        ALGAE(-55);

        private Angle angle;

        private State(Angle angle) {
            this.angle = angle;
        }
        private State(double degrees) {
            this(Units.Degrees.of(degrees));
        }

        public Angle angle() {
            return angle;
        }
    }
    
    public Arm() {
        wrist = new TalonFX(9);
        take = new SparkFlex(16, MotorType.kBrushless);
        
        // wrist.setNeutralMode(NeutralModeValue.Brake);
        wrist.getConfigurator().apply(ArmConstants.wristConfig);

        SparkFlexConfig config = new SparkFlexConfig();
        config
            .idleMode(IdleMode.kCoast)
            .inverted(false)
            .smartCurrentLimit(30, 25);
        // config.softLimit
        //     .forwardSoftLimitEnabled(false)
        //     .reverseSoftLimitEnabled(false)
        //     .forwardSoftLimit(0)
        //     .reverseSoftLimit(0);
        // config.closedLoop.pidf(target, target, target, target)
        take.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        wristRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                voltage -> {
                    wrist.setVoltage(voltage.in(Units.Volts));
                },
                log -> {
                    log.motor("wrist")
                        .voltage(wrist.getMotorVoltage().getValue())
                        .angularPosition(getPosition())
                        .angularVelocity(getVelocity())
                        .angularAcceleration(getAcceleration())
                        .current(getWristCurrent());
                },
                this
            )
        );
    }

    public Angle getTarget() {
        return state.angle;
    }

    // public double getAngle() {
    //     return wrist.getPosition().getValueAsDouble();
    // }
    // public double getVelocity() {
    //     return wrist.getVelocity().getValueAsDouble();
    // }
    public Angle getPosition() {
        return wrist.getPosition().getValue().times(WRIST_RATIO).plus(ArmConstants.START_HORIZONTAL_OFFSET);
    }
    public AngularVelocity getVelocity() {
        return wrist.getVelocity().getValue().times(WRIST_RATIO);
    }
    public AngularAcceleration getAcceleration() {
        return wrist.getAcceleration().getValue().times(WRIST_RATIO);
    }
    public Current getWristCurrent() {
        return wrist.getSupplyCurrent().getValue();
    }

    @Override
    public void periodic() {
        // motor.set(ElevatorConstants.pidConroller.calculate(getPosition(), getTarget()) + ElevatorConstants.feedforward.calculate())
        // SmartDashboard.putNumber("Wrist Target", getTarget());
        // SmartDashboard.putNumber("Wrist Position", getPosition());
        // SmartDashboard.putNumber("position 16", take.getEncoder().getPosition());
        SmartDashboard.putNumber("Wrist Current (A)", getWristCurrent().in(Units.Amp));
        SmartDashboard.putNumber("Wrist Power", wrist.get());
        SmartDashboard.putNumber("Wrist Position", getPosition().in(Units.Degree));
        SmartDashboard.putNumber("Wrist Velo", getVelocity().in(Units.DegreesPerSecond));
        SmartDashboard.putNumber("Wrist Accel", getAcceleration().in(Units.DegreesPerSecondPerSecond));
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
            () -> controller.reset(getPosition().in(Units.Degrees), getVelocity().in(Units.DegreesPerSecond)),
            () -> wrist.set(
                controller.calculate(getPosition().in(Units.Degrees), getTarget().in(Units.Degrees))
                    + ArmConstants.feedforward.calculate(getPosition().in(Units.Radians), controller.getSetpoint().velocity) / RobotController.getBatteryVoltage()
            ),
            (interrupted) -> wrist.set(0),
            controller::atGoal,
            this
        );
    }

    public void runTakeRaw(double power) {
        take.set(power);
    }
    public void runWristRaw(double power) {
        wrist.set(power + ArmConstants.feedforward.calculate(getPosition().in(Units.Radians), power) / RobotController.getBatteryVoltage());
    }



    //--------------------------------------------------------------------------
    // Sys ID Stuff
    private final SysIdRoutine wristRoutine;

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return wristRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return wristRoutine.dynamic(direction);
    }
}
