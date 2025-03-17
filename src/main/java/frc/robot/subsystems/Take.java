package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.ExponentialLowPassFilter;

public class Take extends SubsystemBase {
    private SparkFlex take;
    private double originalCurrent;
    private double filteredCurrent;

    private ExponentialLowPassFilter filter = new ExponentialLowPassFilter(0.008, 1.7); // 0.00012, 2.8

    public Take() {
        take = new SparkFlex(16, MotorType.kBrushless);


        SparkFlexConfig config = new SparkFlexConfig();
        config
            .idleMode(IdleMode.kCoast)
            .inverted(false)
            .smartCurrentLimit(60, 50);
        
        take.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void periodic() {
        originalCurrent = take.getOutputCurrent();
        filteredCurrent = filter.update(originalCurrent);

        SmartDashboard.putNumber("Take Current", originalCurrent);
        SmartDashboard.putNumber("Take Current w/ Low Pass Filter", filteredCurrent);
    }


    public Command outtake(){
        return new FunctionalCommand(
            () -> filteredCurrent=57,
            () -> runTakeRaw(-1),
            (x) -> runTakeRaw(0),
            () -> filteredCurrent<=45,
            this
        );
    }
    public Command intake(){
        return new FunctionalCommand(
            () -> filteredCurrent=45,
            () -> runTakeRaw(1),
            (x) -> runTakeRaw(0),
            () -> filteredCurrent>=57,
            this
        );
    }
    public Command runTake(DoubleSupplier power) {
        return run(() -> runTakeRaw(power.getAsDouble()));
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
}
// package frc.robot.subsystems;

// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;

// import java.util.function.DoubleSupplier;

// import com.revrobotics.spark.SparkFlex;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.config.SparkFlexConfig;

// import edu.wpi.first.math.estimator.KalmanFilter;
// import edu.wpi.first.math.filter.MedianFilter;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class Take extends SubsystemBase {
//     private SparkFlex take;
//     private MedianFilter mFilter = new MedianFilter(10);
    

//     public Take() {
//         take = new SparkFlex(16, MotorType.kBrushless);

//         SparkFlexConfig config = new SparkFlexConfig();
//         config
//             .idleMode(IdleMode.kCoast)
//             .inverted(false)
//             .smartCurrentLimit(60, 50);
//         // config.softLimit
//         //     .forwardSoftLimitEnabled(false)
//         //     .reverseSoftLimitEnabled(false)
//         //     .forwardSoftLimit(0)
//         //     .reverseSoftLimit(0);
//         // config.closedLoop.pidf(target, target, target, target)
//         take.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

//     }

//     @Override
//     public void periodic() {
//         double current = mFilter.calculate(take.getOutputCurrent());
//         SmartDashboard.putNumber("Take Current", take.getOutputCurrent());
//         SmartDashboard.putNumber("Take Current w/ medianFilter", current);
//     }


//     public Command runTake(DoubleSupplier power) {
//         return run(() -> runTakeRaw(power.getAsDouble()));
//     }
//     public Command runIntake(DoubleSupplier power) {
//         return run(() -> runTakeRaw(power.getAsDouble()));
//     }
//     public Command runOuttake(DoubleSupplier power) {
//         return run(() -> runTakeRaw(-power.getAsDouble()));
//     }


//     public void runTakeRaw(double power) {
//         take.set(power);
//     }
// }
