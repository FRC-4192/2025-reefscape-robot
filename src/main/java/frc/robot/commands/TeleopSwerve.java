package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.JerkFilter;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.DriverConstants;

@SuppressWarnings("unused")
public class TeleopSwerve extends Command {
    private final SwerveDrive swerve;
    private final DoubleSupplier drive;
    private final DoubleSupplier strafe;
    private final DoubleSupplier turn;
    private final BooleanSupplier isFieldCentric;
    private final BooleanSupplier isSlow;

//    private boolean isFieldCentric.getAsBoolean() = true;
//    private boolean slowMode = false;
//    private double maxSpeed = DriverConstants.swerveMaxTransSpeed;

    private final JerkFilter tipFilterX = new JerkFilter(DriverConstants.tipConstraints);
    private final JerkFilter tipFilterY = new JerkFilter(DriverConstants.tipConstraints);
    private final SlewRateLimiter limiter = new SlewRateLimiter(10, -20, 0);

    public TeleopSwerve(SwerveDrive swerve, DoubleSupplier drive, DoubleSupplier strafe, DoubleSupplier turn) {
        this(swerve, drive, strafe, turn, () -> true, () -> false);
    }
            
    public TeleopSwerve(SwerveDrive swerve, DoubleSupplier drive, DoubleSupplier strafe, DoubleSupplier turn,
                        BooleanSupplier isFieldCentric, BooleanSupplier isSlow) {
        super();
        this.swerve = swerve;
        this.drive = drive;
        this.strafe = strafe;
        this.turn = turn;
        this.isFieldCentric = isFieldCentric;
        this.isSlow = isSlow;
        
        addRequirements(swerve);
    }

//    public void toggleSlow() {
//        slowMode = !slowMode;
//    }

    @Override
    public void execute() {
//        if (isFieldCentric.getAsBoolean())
//            isFieldCentric.getAsBoolean() = !isFieldCentric.getAsBoolean();

        double translationVal = tipFilterX.calculate(MathUtil.applyDeadband(-drive.getAsDouble(), DriverConstants.stickDeadband));
        double strafeVal = tipFilterY.calculate(MathUtil.applyDeadband(-strafe.getAsDouble(), DriverConstants.stickDeadband));
        double rotationVal = MathUtil.applyDeadband(-turn.getAsDouble(), DriverConstants.stickDeadband);

        swerve.drive(
            new Pose2d(
                new Translation2d(
                        isFieldCentric.getAsBoolean() ? translationVal : -translationVal,
                        isFieldCentric.getAsBoolean() ? strafeVal : -strafeVal
                ).times(Constants.SwerveConstants.maxSpeed * (isSlow.getAsBoolean() ? DriverConstants.swerveSlowSpeed : DriverConstants.swerveMaxTransSpeed)),
                Rotation2d.fromRadians(rotationVal).times(Constants.SwerveConstants.maxAngularVelocity * (isSlow.getAsBoolean() ? DriverConstants.swerveSlowSpeed : DriverConstants.swerveMaxTurnSpeed))
            ),
            isFieldCentric.getAsBoolean(),
            true
        );

        SmartDashboard.putBoolean("Field Centric", isFieldCentric.getAsBoolean());
        SmartDashboard.putBoolean("Slow Mode", isSlow.getAsBoolean());
        SmartDashboard.putNumberArray("DriverX", new double[] {-drive.getAsDouble(), translationVal});
    }
}
