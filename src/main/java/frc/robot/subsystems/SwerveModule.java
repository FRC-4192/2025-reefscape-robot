package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import static frc.robot.Constants.SwerveConstants;


public class SwerveModule {
    private TalonFX driveMotor;
    private TalonFX steerMotor;
    private CANcoder    absoluteEncoder;
    // private RelativeEncoder driveEncoder;
    // private RelativeEncoder steerEncoder;

    private Rotation2d angleOffset;

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    /* angle motor control requests */
    private final PositionVoltage anglePosition = new PositionVoltage(0);

    
    public SwerveModule(SwerveConstants.ModuleConstants moduleConstants) {
        angleOffset = moduleConstants.angleOffset;
        driveMotor = new TalonFX(moduleConstants.driveMotorID);
        steerMotor = new TalonFX(moduleConstants.steerMotorID);
        absoluteEncoder = new CANcoder(moduleConstants.absoluteEncoderID);
                
        absoluteEncoder.getConfigurator().apply(SwerveConstants.swerveCANcoderConfig);

        steerMotor.getConfigurator().apply(SwerveConstants.swerveSteerFXConfig);

        driveMotor.getConfigurator().apply(SwerveConstants.swerveDriveFXConfig);
        driveMotor.getConfigurator().setPosition(0.0);

        resetToAbsolute();
    }

    
    /**
    Get the distance in meters.
    */
    public double getDistance() {
        return driveMotor.getPosition().getValueAsDouble() * SwerveConstants.wheelCircumference;
    }
    
    /**
    Get the angle.
    */
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(steerMotor.getPosition().getValueAsDouble());
    }

    public Rotation2d getAbsoluteAngle() {
        return Rotation2d.fromRotations(absoluteEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDistance(), getAngle());
    }

    public LinearVelocity getVelocity() {
        return driveMotor.getVelocity().getValue().asFrequency().times(Distance.ofBaseUnits(SwerveConstants.wheelCircumference, Units.Meter));
    }    

    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getAngle());
    }

    public void resetToAbsolute() {
        double absolutePosition = getAbsoluteAngle().minus(angleOffset).getRotations(); //todo not sure if we should flip it yet
        steerMotor.setPosition(absolutePosition);
    }
    
    /**
    Set the swerve module state.
    @param state The swerve module state to set.
    */
    public void setState(SwerveModuleState state, boolean isOpenLoop) {
        steerMotor.setControl(anglePosition.withPosition(state.angle.getRotations()));
        if(isOpenLoop) {
            driveMotor.setControl(driveDutyCycle.withOutput(state.speedMetersPerSecond / SwerveConstants.maxSpeed));
        } else {
            driveMotor.setControl(
                driveVelocity.withVelocity(state.speedMetersPerSecond / SwerveConstants.wheelCircumference)
                             .withFeedForward(SwerveConstants.driveFeedForward.calculate(state.speedMetersPerSecond))
            );
        }
    }

}
