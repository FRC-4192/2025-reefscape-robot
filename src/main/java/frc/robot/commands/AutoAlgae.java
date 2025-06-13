// package frc.robot.commands;

// import org.dyn4j.dynamics.joint.PulleyJoint;

// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.FunctionalCommand;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.subsystems.Arm;
// import frc.robot.subsystems.SwerveDrive;

// public class AutoAlgae extends Command{
//     private final Arm arm;
//     private final TargetAlign targetAlign;
//     private final SwerveDrive swerve;
//     private final Command driveBack;

//     private Timer timer= new Timer();

//     public AutoAlgae(Arm arm, SwerveDrive swerve){
//         this.targetAlign = new TargetAlign(swerve, 0);
//         this.arm=arm;
//         this.swerve=swerve;
//         this.driveBack = new FunctionalCommand(
//                     () -> timer.reset(),
//                     () -> swerve.drive(new ChassisSpeeds(-1,0,0)),
//                     () -> swerve.drive(new ChassisSpeeds(0,0,0)),
//                     timer::hasElapsed(1.1),
//                     swerve
//                 );

//     }

//     public void initialize(){
//         if(targetAlign!=null){
//             targetAlign.schedule();
//         }
//         if(arm!=null){
//             arm.setTargetOnly(Arm.State.INTAKING).schedule();
//         }
//     }

//     public void execute(){
//         new ParallelCommandGroup(
//             arm.setTargetOnly(Arm.State.HOLDING).asProxy(),
//             new WaitCommand(0.5).andThen().asProxy()
//         ).schedule();
        
//     }

    
    
// }
