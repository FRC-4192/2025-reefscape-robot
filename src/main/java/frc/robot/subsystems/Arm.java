package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private TalonFX wrist;
    private SparkFlex take;
    
    public Arm() {
        wrist = new TalonFX(9);
        take = new SparkFlex(16, MotorType.kBrushless);
        
        // wrist.setNeutralMode(NeutralModeValue.Coast);

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
    }

    @Override
    public void periodic() {
        // motor.set(ElevatorConstants.pidConroller.calculate(getPosition(), getTarget()) + ElevatorConstants.feedforward.calculate())
        // SmartDashboard.putNumber("Wrist Target", getTarget());
        // SmartDashboard.putNumber("Wrist Position", getPosition());
        SmartDashboard.putNumber("position 16", take.getEncoder().getPosition());
        SmartDashboard.putNumber("Wrist Current (A)", take.getOutputCurrent());
        SmartDashboard.putNumber("Wrist Power", take.get());
    }

    public void runRaw(double power) {
        take.set(power);
    }
}
