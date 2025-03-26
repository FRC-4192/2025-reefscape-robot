package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
// import frc.robot.subsystems.RampTake;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Take;

public class AutoScore extends Command {
    private final Elevator elevator;
    private final Arm arm;
    private final Take take;
    private final Command prepScore;
    private final Command scoreCoral;
    private final TargetAlign align;


    public AutoScore(TargetAlign align, Elevator elevator, Arm arm, Take take) {
        this.align = align;
        this.elevator = elevator;
        this.arm = arm;
        this.take = take;
        prepScore = elevator.setTarget(Elevator.State.L4).raceWith(new WaitCommand(3));

        scoreCoral = new SequentialCommandGroup(
            arm.setTarget(Arm.State.SCORING).raceWith(new WaitCommand(1)),
            take.runOuttake(()-> 1).raceWith(new WaitCommand(1.5)),
            take.runOuttake(()-> 0).raceWith(new WaitCommand(.3)),
            arm.setTarget(Arm.State.HOLDING).raceWith(new WaitCommand(1)),
            elevator.setTarget(Elevator.State.L0)
        );


        for (Subsystem s : new Subsystem[] {elevator, arm, take}) {
            if (s != null)
                addRequirements(s);
        }
    }
    // public AutoScore(TargetAlign align) {
    //     this(align, null, null, null);
    // }

    @Override
    public void initialize() {
        align.initialize();
    }

    @Override
    public boolean isFinished() {
        return scoreCoral.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        scoreCoral.end(interrupted);
    }
    @Override
    public void execute() {
        align.execute();
        prepScore.execute();
        if (elevator.getError()<.05){

        }
 
    }
}
