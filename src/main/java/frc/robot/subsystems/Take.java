package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.ExponentialLowPassFilter;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.IntakeConstants;

public class Take extends SubsystemBase {
    private final SparkFlex take;
    // private final TalonFX take;
    private double originalCurrent;
    private double filteredCurrent;

    private ExponentialLowPassFilter filter = new ExponentialLowPassFilter(0.008, 1.7); // 0.00012, 2.8
    private boolean spike;
        
    public Take() {
        take = new SparkFlex(16, MotorType.kBrushless);
        take.configure(IntakeConstants.motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // take = new TalonFX(16);
        // take.getConfigurator().apply(IntakeConstants.takeConfig);

    }

    @Override
    public void periodic() {
        originalCurrent = take.getOutputCurrent();
        // originalCurrent = take.getTorqueCurrent().getValueAsDouble();
        filteredCurrent = filter.update(originalCurrent);

        SmartDashboard.putNumber("Take Current", originalCurrent);
        SmartDashboard.putNumber("Take Current w/ Low Pass Filter", filteredCurrent);
        SmartDashboard.putNumber("Take Velocity", take.getEncoder().getVelocity());
        // SmartDashboard.putNumber("Take Velocity", take.getVelocity().getValueAsDouble());


    }


    public Command outtakeCoral(double power){
        spike=false;
        return new FunctionalCommand(
            () -> {},
            () -> {
                runRaw(-power);
                spike = spike || filteredCurrent >= 50;
            },
            (x) -> runRaw(0),
            () -> filteredCurrent <= 45 && spike,
            this
        );
    }
    public Command intakeCoral(DoubleSupplier power) {
        return new FunctionalCommand(
            () -> {},
            () -> {
                runRaw(power.getAsDouble());
            },
            (x) -> runRaw(x ? power.getAsDouble() : 0),
            () -> filteredCurrent >= 45,
            this
        );
    }

    public Command runIntake(DoubleSupplier power) {
        return run(() -> runRaw(power.getAsDouble()));
    }
    public Command runOuttake(DoubleSupplier power) {
        return run(() -> runRaw(-power.getAsDouble()));
    }

    public void runRaw(double power) {
        take.set(power);
    }

    public Command runTakeOnce(double power) {
        return runOnce(() -> runRaw(power));
    }
}
    

