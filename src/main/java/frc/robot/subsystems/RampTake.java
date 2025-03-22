// package frc.robot.subsystems;

// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;

// import java.util.function.DoubleSupplier;

// import com.revrobotics.spark.SparkFlex;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.config.SparkFlexConfig;

// import edu.wpi.first.math.estimator.KalmanFilter;
// import edu.wpi.first.math.Matrix;
// import edu.wpi.first.math.Nat;
// import edu.wpi.first.math.numbers.N1;
// import edu.wpi.first.math.system.plant.LinearSystemId;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.FunctionalCommand;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.lib.util.ExponentialLowPassFilter;

// public class RampTake extends SubsystemBase {
//     private SparkFlex take;
//     private double originalCurrent;
//     private double filteredCurrent;

//     private ExponentialLowPassFilter filter = new ExponentialLowPassFilter(0.5, 1);

//     public RampTake() {
//         take = new SparkFlex(17, MotorType.kBrushless);


//         SparkFlexConfig config = new SparkFlexConfig();
//         config
//             .idleMode(IdleMode.kCoast)
//             .inverted(true)
//             .smartCurrentLimit(60, 50);
        
//         take.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
//     }

//     @Override
//     public void periodic() {
//         originalCurrent = take.getOutputCurrent();
//         filteredCurrent = filter.update(originalCurrent);

//         SmartDashboard.putNumber("Take Current", originalCurrent);
//         SmartDashboard.putNumber("Take Current w/ Low Pass Filter", filteredCurrent);
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

//     public Command runTakeOnce(double power) {
//         return runOnce(() -> runTakeRaw(power));
//     }
// }
