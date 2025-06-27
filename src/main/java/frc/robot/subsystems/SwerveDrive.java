package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.LimelightHelpers;
import frc.lib.util.SwerveModuleStatesDashboarder;
import frc.robot.Constants;
import frc.robot.Constants.LimelightConstants;

import static frc.robot.Constants.SwerveConstants;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SwerveDrive extends SubsystemBase {

    // Attributes
    SwerveDriveOdometry   odometry;
    AHRS                  gyro; // Psuedo-class representing a gyroscope.
    SwerveModule[]        swerveModules; // Psuedo-class representing swerve modules.

    SwerveDrivePoseEstimator poseEstimator; // to replace odo bc better

    public Field2d fieldO = new Field2d();
    public Field2d fieldV = new Field2d();
    public SwerveModuleStatesDashboarder smsd = new SwerveModuleStatesDashboarder();
    private ChassisSpeeds lastChassisSpeeds = new ChassisSpeeds();

    private boolean braked = false;
    
    // Constructor
    public SwerveDrive() {
        swerveModules = new SwerveModule[] {
            new SwerveModule(SwerveConstants.moduleConstants[0]),
            new SwerveModule(SwerveConstants.moduleConstants[1]),
            new SwerveModule(SwerveConstants.moduleConstants[2]),
            new SwerveModule(SwerveConstants.moduleConstants[3])
        };
        
        gyro = new AHRS(NavXComType.kMXP_SPI);
        resetGyro();

        // Create the SwerveDriveOdometry given the current angle, the robot is at x=0, r=0, and heading=0
        odometry = new SwerveDriveOdometry(
            SwerveConstants.kinematics,
            Rotation2d.fromDegrees(gyro.getAngle()), // returns current gyro reading as a Rotation2d
            new SwerveModulePosition[]{new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()},
            // Front-Left, Front-Right, Back-Left, Back-Right
            Pose2d.kZero // x=0, y=0, heading=0
        );
        poseEstimator = new SwerveDrivePoseEstimator(
            SwerveConstants.kinematics,
            getHeadingGyro(),
            getModulePositions(),
            Pose2d.kZero
        );

        AutoBuilder.configure(
            this::getPoseV,
            (x) -> {setPose(x);setPoseV(x);},
            this::getCurrentVelocity,
            this::driveAuto, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            SwerveConstants.autoFollowerController,
            SwerveConstants.autoPathFollowerConfig,
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this
        );

        SmartDashboard.putData("FieldO", fieldO);
        SmartDashboard.putData("FieldV", fieldV);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }
    public Pose2d getPoseV() {
        return poseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        odometry.resetPosition(getHeadingGyro(), getModulePositions(), pose);
    }
    public void setPoseV(Pose2d pose) {
        poseEstimator.resetPosition(getHeadingGyro(), getModulePositions(), pose);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }
    public Rotation2d getHeadingV() {
        return getPoseV().getRotation();
    }

    public void setHeading(Rotation2d heading) {
        odometry.resetPosition(getHeadingGyro(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }
    public void setHeadingV(Rotation2d heading) {
        setPoseV(new Pose2d(getPoseV().getTranslation(), heading));
    }

    public void zeroHeading() {
        setHeading(Rotation2d.kZero);
    }
    public void zeroHeadingV() {
        setHeadingV(Rotation2d.kZero);
    }

    public Rotation2d getHeadingGyro() {
        return Rotation2d.fromDegrees(-gyro.getYaw()); // roll since roborio/navx is mounted vertically
        // return Constants.Swerve.invertGyro ? Rotation2d.fromDegrees(360- gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    // public void resetModulesToAbsolute() {
    //     for(SwerveModule mod : swerveMods){
    //         mod.resetToAbsolute();
    //     }
    // }

    public void resetGyro() {
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
        return new SwerveModulePosition[] {
            swerveModules[0].getPosition(),
            swerveModules[1].getPosition(),
            swerveModules[2].getPosition(),
            swerveModules[3].getPosition()
        };
    }

    // @AutoLogOutput
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            swerveModules[0].getState(),
            swerveModules[1].getState(),
            swerveModules[2].getState(),
            swerveModules[3].getState()
        };
    }

    private void setSwerveModuleStates(SwerveModuleState[] states, boolean isOpenLoop) {
        for (int i = 0; i < 4; i ++) {
            swerveModules[i].setState(states[i], isOpenLoop);
        }
    }

    public void driveAuto(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] swerveModuleStates = SwerveConstants.kinematics.toSwerveModuleStates(chassisSpeeds);
        SmartDashboard.putString("aut cs", chassisSpeeds.toString());
        lastChassisSpeeds = chassisSpeeds;

        setSwerveModuleStates(swerveModuleStates, false);
    }
    public void drive(ChassisSpeeds chassisSpeeds) {
        drive(chassisSpeeds, true);
    }

    public void drive(ChassisSpeeds chassisSpeeds, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = SwerveConstants.kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxSpeed);
        lastChassisSpeeds = chassisSpeeds;

        setSwerveModuleStates(swerveModuleStates, isOpenLoop);
    }

    public void drive(Pose2d target, boolean fieldRelative, boolean isOpenLoop) {
        drive(fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(target.getX(), target.getY(), target.getRotation().getRadians(), getHeading())
                : new ChassisSpeeds(target.getX(), target.getY(), target.getRotation().getRadians()),
            isOpenLoop
        );
    }

    public void lockDrive() {
        setSwerveModuleStates(new SwerveModuleState[] {
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
        }, false);
    }

    public void toggleBrakes() {
        braked = !braked;
        for (SwerveModule module : swerveModules)
            module.setBrake(braked);
    }
    
    
    @Override
    public void periodic() {
        odometry.update(getHeadingGyro(), getModulePositions());
        poseEstimator.update(getHeadingGyro(), getModulePositions());
        updateVisionOdo();

        for(int i = 0; i < 4; i ++){
            SmartDashboard.putNumber("Mod" + i + " encoder (°)", swerveModules[i].getAbsoluteAngle().getDegrees());
            // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
        // SmartDashboard.putNumber("Swerve Rotation Target", rotationTarget);
        // SmartDashboard.putNumber("Swerve Translation Target", translationTarget.getNorm());
        SmartDashboard.putNumber("Gyro Yaw", getHeadingGyro().getDegrees());
        SmartDashboard.putNumber("Odo yaw", getHeading().getDegrees());
        SmartDashboard.putNumber("Odo posX", getPose().getMeasureX().in(Units.Meters));
        SmartDashboard.putNumber("Odo posY", getPose().getMeasureY().in(Units.Meters));
        
        SmartDashboard.putNumber("OdoV yaw", getHeadingV().getDegrees());
        SmartDashboard.putNumber("OdoV posX", getPoseV().getMeasureX().in(Units.Meters));
        SmartDashboard.putNumber("OdoV posY", getPoseV().getMeasureY().in(Units.Meters));

        SmartDashboard.putNumberArray("swerve module states", smsd.update(getModuleStates()));
        // SmartDashboard.putNumberArray("chassis speeds", new double[] {lastChassisSpeeds.vxMetersPerSecond, lastChassisSpeeds.vyMetersPerSecond, lastChassisSpeeds.omegaRadiansPerSecond});
        Logger.recordOutput("swerve module states", getModuleStates());
        Logger.recordOutput("swerve chassis speeds", lastChassisSpeeds);
        Logger.recordOutput("swerve gyro yaw", getHeadingGyro());
        Logger.recordOutput("swerve odo yaw", getHeadingV());

        SmartDashboard.putBoolean("Braked", braked);

        fieldO.setRobotPose(getPose());
        fieldV.setRobotPose(getPoseV());
    }
    

    public void updateVisionOdo() {
        LimelightHelpers.SetRobotOrientation(
            LimelightConstants.reefLimeName,
            getHeadingV().getDegrees(),
            gyro.getRate(),
            gyro.getPitch(),
            0,
            gyro.getRoll(),
            0
        );
        // LimelightHelpers.PoseEstimate vis = LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightConstants.name);
        LimelightHelpers.PoseEstimate vis = LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightConstants.reefLimeName);

        if (vis == null || Math.abs(gyro.getRate()) > 360 || vis.tagCount < 1)
            return;
        
        poseEstimator.addVisionMeasurement(
            new Pose2d(vis.pose.getTranslation(), vis.pose.getRotation().rotateBy(Rotation2d.k180deg)),
            vis.timestampSeconds,
            VecBuilder.fill(vis.avgTagDist * .1, vis.avgTagDist * .1, vis.avgTagDist * .01)
        );
    }
}
