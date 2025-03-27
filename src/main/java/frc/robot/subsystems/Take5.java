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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Take5 extends SubsystemBase {
    private final SparkFlex take;
    private final KalmanFilter<N1, N1, N1> kalmanFilter;
    
    private final Matrix<N1, N1> stateStdDevs = new Matrix<>(Nat.N1(), Nat.N1());
    private final Matrix<N1, N1> measurementStdDevs = new Matrix<>(Nat.N1(), Nat.N1());
    private final double dtSeconds = 1.0 / 50.0;

    public Take5() {
        take = new SparkFlex(16, MotorType.kBrushless);

        SparkFlexConfig config = new SparkFlexConfig();
        config
            .idleMode(IdleMode.kCoast)
            .inverted(false)
            .smartCurrentLimit(60, 50);
        
        take.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        Nat<N1> states = Nat.N1(); // One state: the intake current
        Nat<N1> outputs = Nat.N1(); // One output: the noisy current measurement

        
        kalmanFilter = new KalmanFilter<>(
            states, outputs, 
             LinearSystemId.identifyVelocitySystem(.02, .01),
            stateStdDevs, measurementStdDevs, dtSeconds);
    }
    @Override
    public void periodic() {
        double noisyCurrent = take.getOutputCurrent();

        Matrix<N1, N1> y = new Matrix<>(Nat.N1(), Nat.N1());
        y.set(0, 0, noisyCurrent);

        Matrix<N1, N1> controlInput = new Matrix<>(Nat.N1(), Nat.N1());
        controlInput.set(0, 0, 0.0); 

        kalmanFilter.correct(controlInput, y);

        Matrix<N1, N1> filteredCurrent = kalmanFilter.getXhat();
        double smoothedCurrent = filteredCurrent.get(0, 0); 

        SmartDashboard.putNumber("Take Current", noisyCurrent);
        SmartDashboard.putNumber("Take Current w/ Kalman Filter", smoothedCurrent);
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
