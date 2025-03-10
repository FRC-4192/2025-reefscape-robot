package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

public class TeleopSwerve extends Command {
    private SwerveDrive swerve;
    private DoubleSupplier drive;
    private DoubleSupplier strafe;
    private DoubleSupplier turn;
    private BooleanSupplier povSwitch;
    private BooleanSupplier slow;

    private boolean fieldRelative = false;

    public TeleopSwerve(SwerveDrive swerve, DoubleSupplier drive, DoubleSupplier strafe, DoubleSupplier turn) {
        this(swerve, drive, strafe, turn, () -> false, () -> false);
    }
            
    public TeleopSwerve(SwerveDrive swerve, DoubleSupplier drive, DoubleSupplier strafe, DoubleSupplier turn,
                        BooleanSupplier povSwitch, BooleanSupplier slow) {
        super();
        this.swerve = swerve;
        this.drive = drive;
        this.strafe = strafe;
        this.turn = turn;
        this.povSwitch = povSwitch;
        this.slow = slow;
        
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public void execute() {
        if (povSwitch.getAsBoolean()) fieldRelative = !fieldRelative;
        
        double translationVal = MathUtil.applyDeadband(-drive.getAsDouble(), Constants.OperatorConstants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafe.getAsDouble(), Constants.OperatorConstants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(turn.getAsDouble(), Constants.OperatorConstants.stickDeadband);

        swerve.drive(
            new Pose2d(
                new Translation2d(translationVal, strafeVal).times(Constants.SwerveConstants.maxSpeed * (slow.getAsBoolean() ? .3 : 1)),
                Rotation2d.fromRadians(rotationVal).times(Constants.SwerveConstants.maxAngularVelocity * (slow.getAsBoolean() ? .3 : 1))
            ),
            fieldRelative,
            true
        );
    }
}
