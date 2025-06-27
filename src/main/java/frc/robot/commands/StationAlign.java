package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.SwerveDrive;

public class StationAlign extends Command {

    private final SwerveDrive swerve;
    private final int pos;
    private final double[] posArr = {-.635, 0, 0.635};
    private final boolean forward;
    private final boolean strafe;
    private final boolean turn;
    
    private final PIDController forwardController = new PIDController(2.5, 0, 0.0001);
    private final PIDController strafeController = new PIDController(2.5, 0, 0.0001);
    private final PIDController rotationController = new PIDController(.06, 0, 0.0001);
    private final double kStatic = .16;

    
    private static final double bumperLength = .914;
    private static final double bumperWidth = .902;

    public StationAlign(SwerveDrive swerve, int pos, boolean forward, boolean strafe, boolean turn) {
        this.swerve = swerve;
        this.pos = pos;
        this.forward = forward;
        this.strafe = strafe;
        this.turn = turn;

        forwardController.setTolerance(0.02, 0.05);
        strafeController.setTolerance(0.01, 0.05);
        rotationController.setTolerance(0.8, 2);

        addRequirements(swerve);
    }
    public StationAlign(SwerveDrive swerve, int pos) {
        this(swerve, pos, true, true, true);
    }

    @Override
    public void initialize() {
        // LimelightHelpers.setPriorityTagID(Constants.Limelight.name, target != -2 ? target :
        // DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == Alliance.Blue ? 7 : 4 : -1);
        forwardController.reset();
        strafeController.reset();
        rotationController.reset();
        LimelightHelpers.setLEDMode_ForceOn(LimelightConstants.coralLimeName);
    }


    @Override
    public void execute() {
        Pose3d pose = LimelightHelpers.getTargetPose3d_RobotSpace(LimelightConstants.coralLimeName);
        if (pose.getTranslation().getNorm() != 0 && pose.getRotation().getAngle() != 0 && pose.getTranslation().getNorm() < 1.5) {
            double rotationTarget = rotationController.calculate(Math.toDegrees(pose.getRotation().getY()), 0);
            double translationTarget= -strafeController.calculate(pose.getX(), posArr[pos]);
            double forwardTarget = forwardController.calculate(pose.getZ(), bumperLength/2);
            swerve.drive(new ChassisSpeeds(
                forward ? -forwardTarget : 0,
                strafe ? -translationTarget : 0,
                turn ? -rotationTarget : 0
            ));
        }
        SmartDashboard.putNumber("station Vision Angle", Math.toDegrees(pose.getRotation().getX()));
        SmartDashboard.putNumber("station Vision Offset", pose.getX());
        SmartDashboard.putNumber("station Vision Dist", pose.getTranslation().getNorm());
        SmartDashboard.putNumber("station Tag X error", forwardController.getError());
        SmartDashboard.putNumber("station Tag Y error", strafeController.getError());
        SmartDashboard.putNumber("station Tag R error", rotationController.getError());
    }

    @Override
    public boolean isFinished() {
        return rotationController.atSetpoint() && strafeController.atSetpoint() && forwardController.atSetpoint();
    }
 
    @Override
    public void end(boolean interrupted) {
        swerve.drive(new ChassisSpeeds());
        if (interrupted)
            LimelightHelpers.setLEDMode_ForceOff(LimelightConstants.coralLimeName);
        else
            LimelightHelpers.setLEDMode_ForceBlink(LimelightConstants.coralLimeName);
    }


}
