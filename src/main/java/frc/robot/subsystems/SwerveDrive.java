package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.pathplanner.lib.auto.AutoBuilder;
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

        // AutoBuilder.configure(
        //     this::getPose,
        //     this::setPose,
        //     this::getCurrentVelocity,
        //     (speeds, feedforwards) -> drive(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        //     SwerveConstants.autoFollowerController,
        //     SwerveConstants.autoPathFollowerConfig,
        //     () -> {
        //         // Boolean supplier that controls when the path will be mirrored for the red alliance
        //         // This will flip the path being followed to the red side of the field.
        //         // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        //         var alliance = DriverStation.getAlliance();
        //         if (alliance.isPresent()) {
        //             return alliance.get() == DriverStation.Alliance.Red;
        //         }
        //         return false;
        //     },
        //     this
        // );
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        odometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading) {
        odometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
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

    // robot relative
    public ChassisSpeeds getCurrentVelocity() {
        return Constants.SwerveConstants.kinematics.toChassisSpeeds(getModuleStates());
    }

    // Fetch the current swerve module positions.
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[]{
            swerveModules[0].getPosition(),
            swerveModules[1].getPosition(),
            swerveModules[2].getPosition(),
            swerveModules[3].getPosition()
        };
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            swerveModules[0].getState(),
            swerveModules[1].getState(),
            swerveModules[2].getState(),
            swerveModules[3].getState()
        };
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
    
    
    @Override
    public void periodic() {
        odometry.update(getGyroYaw(), getModulePositions());

        for(int i = 0; i < 4; i ++){
            SmartDashboard.putNumber("Mod" + i + " encoder (°)", swerveModules[i].getAbsoluteAngle().getDegrees());
            // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
        // SmartDashboard.putNumber("Swerve Rotation Target", rotationTarget);
        // SmartDashboard.putNumber("Swerve Translation Target", translationTarget.getNorm());
    }
    
}
