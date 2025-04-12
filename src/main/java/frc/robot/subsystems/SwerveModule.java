package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.SwerveConstants;


public class SwerveModule {
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANcoder    absoluteEncoder;

    private final Rotation2d angleOffset;

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    /* angle motor control requests */
    private final PositionVoltage anglePosition = new PositionVoltage(0);

    private SwerveModuleState lastState = new SwerveModuleState();

    
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
    public Distance getDistance() {
        return Units.Meters.of(SwerveConstants.wheelCircumference).times(driveMotor.getPosition().getValueAsDouble());
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
        return Units.MetersPerSecond.of(SwerveConstants.wheelCircumference).times(driveMotor.getVelocity().getValueAsDouble());
    }    

    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getAngle());
    }

    public void resetToAbsolute() {
        double absolutePosition = getAbsoluteAngle().minus(angleOffset).getRotations(); //todo not sure if we should flip it yet
        steerMotor.setPosition(absolutePosition);
    }

    public void setBrake(boolean brakedMode) {
        steerMotor.getConfigurator().apply(SwerveConstants.swerveDriveFXConfig.MotorOutput.withNeutralMode(brakedMode ? NeutralModeValue.Brake : NeutralModeValue.Coast));
    }
    
    /**
    Set the swerve module state.
    @param state The swerve module state to set.
    */
    public void setState(SwerveModuleState state, boolean isOpenLoop) {
        state.optimize(lastState.angle);
        steerMotor.setControl(anglePosition.withPosition(state.angle.getRotations()));
        if(isOpenLoop) {
            driveMotor.setControl(driveDutyCycle.withOutput(state.speedMetersPerSecond / SwerveConstants.maxSpeed));
        } else {
            driveMotor.setControl(
                driveVelocity.withVelocity(state.speedMetersPerSecond / SwerveConstants.wheelCircumference)
                             .withFeedForward(SwerveConstants.driveFeedForward.calculate(state.speedMetersPerSecond))
            );
        }

        lastState = state;
        SmartDashboard.putNumber("driv mot" + driveMotor.getDeviceID(), driveMotor.get());
    }

}
