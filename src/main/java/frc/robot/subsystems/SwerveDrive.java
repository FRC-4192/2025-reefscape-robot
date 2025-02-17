package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import static frc.robot.Constants.SwerveConstants;

public class SwerveDrive extends SubsystemBase {

    // Attributes
    SwerveDriveOdometry   odometry;
    AHRS                  gyro; // Psuedo-class representing a gyroscope.
    SwerveModule[]        swerveModules; // Psuedo-class representing swerve modules.
    
    // Constructor
    public SwerveDrive() {
        swerveModules = new SwerveModule[] {
            new SwerveModule(SwerveConstants.moduleConstants[0]),
            new SwerveModule(SwerveConstants.moduleConstants[1]),
            new SwerveModule(SwerveConstants.moduleConstants[2]),
            new SwerveModule(SwerveConstants.moduleConstants[3])
        };
        
        gyro = new AHRS(NavXComType.kMXP_SPI);

        // Create the SwerveDriveOdometry given the current angle, the robot is at x=0, r=0, and heading=0
        odometry = new SwerveDriveOdometry(
            SwerveConstants.kinematics,
            Rotation2d.fromDegrees(gyro.getAngle()), // returns current gyro reading as a Rotation2d
            new SwerveModulePosition[]{new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()},
            // Front-Left, Front-Right, Back-Left, Back-Right
            Pose2d.kZero // x=0, y=0, heading=0
        );
            
    }

    public Pose2d getPose() {
        Pose2d pose = odometry.getPoseMeters();
        return pose;
    }

    public void setPose(Pose2d pose) {
        odometry.resetPosition(getGyroYaw(), getCurrentSwerveModulePositions(), pose);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading) {
        odometry.resetPosition(getGyroYaw(), getCurrentSwerveModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        setHeading(Rotation2d.kZero);
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getAngle());
        // return Constants.Swerve.invertGyro ? Rotation2d.fromDegrees(360- gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    // public void resetModulesToAbsolute() {
    //     for(SwerveModule mod : swerveMods){
    //         mod.resetToAbsolute();
    //     }
    // }

    public void resetGyro(){
        gyro.zeroYaw();
        gyro.resetDisplacement();
        gyro.reset();
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        drive(chassisSpeeds, true);
    }

    public void drive(ChassisSpeeds chassisSpeeds, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = SwerveConstants.kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxSpeed);

        for(int i = 0; i < 4; i ++) {
            swerveModules[i].setState(swerveModuleStates[i], isOpenLoop);
        }
    }

    public void drive(Pose2d target, boolean fieldRelative, boolean isOpenLoop) {
        drive(fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(target.getX(), target.getY(), target.getRotation().getRadians(), getHeading())
                : new ChassisSpeeds(target.getX(), target.getY(), target.getRotation().getRadians()),
            isOpenLoop
        );
    }
    
    // Fetch the current swerve module positions.
    public SwerveModulePosition[] getCurrentSwerveModulePositions() {
        return new SwerveModulePosition[]{
            swerveModules[0].getModulePosition(),
            swerveModules[1].getModulePosition(),
            swerveModules[2].getModulePosition(),
            swerveModules[3].getModulePosition()
        };
    }
    
    @Override
    public void periodic() {
        odometry.update(getGyroYaw(), getCurrentSwerveModulePositions());

        for(int i = 0; i < 4; i ++){
            SmartDashboard.putNumber("Mod" + i + " encoder (°)", swerveModules[i].getAbsoluteAngle().getDegrees());
            // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
        // SmartDashboard.putNumber("Swerve Rotation Target", rotationTarget);
        // SmartDashboard.putNumber("Swerve Translation Target", translationTarget.getNorm());
    }
    
}
