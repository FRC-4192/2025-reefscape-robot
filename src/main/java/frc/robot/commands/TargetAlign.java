package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.SwerveDrive;

public class TargetAlign extends Command {
    private final SwerveDrive swerve;
    private final boolean rightSide;
    private final boolean forward;
    private final boolean strafe;
    private final boolean turn;

    private final PIDController forwardController = new PIDController(2.5, 0, 0.0001);
    private final PIDController strafeController = new PIDController(2.5, 0, 0.0001);
    private final PIDController rotationController = new PIDController(.06, 0, 0.0001);
    private final double kStatic = .16;

    private static final double bumperLength = .914;
    private static final double bumperWidth = .902;
    private static final double reefSeparation = .33;
        
    public TargetAlign(SwerveDrive swerve, boolean rightSide, boolean forward, boolean strafe, boolean turn) {
        this.swerve = swerve;
        this.rightSide = rightSide;
        this.forward = forward;
        this.strafe = strafe;
        this.turn = turn;

        forwardController.setTolerance(0.02, 0.05);
        strafeController.setTolerance(0.02, 0.05);
        rotationController.setTolerance(0.8, 2);

        addRequirements(swerve);
    }
    public TargetAlign(SwerveDrive swerve, boolean rightSide) {
        this(swerve, rightSide, true, true, true);
    }
    
    @Override
    public void initialize() {
        // LimelightHelpers.setPriorityTagID(Constants.Limelight.name, target != -2 ? target :
        // DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == Alliance.Blue ? 7 : 4 : -1);
        forwardController.reset();
        strafeController.reset();
        rotationController.reset();
        LimelightHelpers.setLEDMode_ForceOn(LimelightConstants.name);
    }

    @Override
    public void execute() {
        Pose3d pose = LimelightHelpers.getTargetPose3d_RobotSpace(LimelightConstants.name);
        if (pose.getTranslation().getNorm() != 0 && pose.getRotation().getAngle() != 0 && pose.getTranslation().getNorm() < 1.5) {
            double rotationTarget = rotationController.calculate(Math.toDegrees(pose.getRotation().getY()), 0);
            double translationTarget = -strafeController.calculate(pose.getX(), .5 * (rightSide ? -reefSeparation : reefSeparation));
            double forwardTarget = forwardController.calculate(pose.getZ(), bumperLength/2);
            swerve.drive(new ChassisSpeeds(
                forward ? forwardTarget : 0,
                strafe ? translationTarget : 0,
                turn ? rotationTarget : 0
            ));
        }
        SmartDashboard.putNumber("Vision Angle", Math.toDegrees(pose.getRotation().getX()));
        SmartDashboard.putNumber("Vision Offset", pose.getX());
        SmartDashboard.putNumber("Vision Dist", pose.getTranslation().getNorm());
    }

    @Override
    public boolean isFinished() {
        return rotationController.atSetpoint() && strafeController.atSetpoint() && forwardController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new ChassisSpeeds());
        LimelightHelpers.setLEDMode_ForceOff(LimelightConstants.name);
    }
}
