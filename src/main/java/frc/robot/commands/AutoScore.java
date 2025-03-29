package frc.robot.commands;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
// import frc.robot.subsystems.RampTake;
import frc.robot.subsystems.Take;

public class AutoScore extends Command {
    private final Elevator elevator;
    private final Arm arm;
    private final Take take;
    private final Command prepScore;
    private final Command scoreCoral;
    private final TargetAlign align;

    public int state = 0;
    private Timer timer = new Timer();

    public AutoScore(TargetAlign align, Elevator elevator, Arm arm, Take take) {
        this.align = align;
        this.elevator = elevator;
        this.arm = arm;
        this.take = take;
        prepScore = elevator.setTargetStay(Elevator.State.L4).raceWith(new WaitCommand(3));

        scoreCoral = new SequentialCommandGroup(
            arm.setTargetStay(Arm.State.SCORING).raceWith(new WaitCommand(1)),
            take.runOuttake(()-> 1).raceWith(new WaitCommand(1.5)),
            take.runOuttake(()-> 0).raceWith(new WaitCommand(.3)),
            arm.setTargetStay(Arm.State.HOLDING).raceWith(new WaitCommand(1)),
            elevator.setTargetStay(Elevator.State.L0)
        );

        // for (Subsystem s : new Subsystem[] {elevator, arm, take}) {
        //     if (s != null)
        //         addRequirements(s);
        // }
    }
    // public AutoScore(TargetAlign align) {
    //     this(align, null, null, null);
    // }

    @Override
    public void initialize() {
        state = 0;
        if (align != null)
            align.schedule();
        if (elevator != null)
            elevator.setTargetStay(Elevator.State.L4).schedule();
        if (arm != null)
            arm.setTargetStay(Arm.State.HOLDING).schedule();;
    }

    @Override
    public boolean isFinished() {
        return state == 4 && Math.abs(elevator.getError().in(Units.Meters)) < .1;
    }

    @Override
    public void end(boolean interrupted) {
        // scoreCoral.end(interrupted);
    }

    @Override
    public void execute() {
        if (state == 0 && align.isFinished() && Math.abs(elevator.getError().in(Units.Meters)) < .05) {
            arm.setTargetOnly(Arm.State.SCORING).schedule();
            state++;
        } else if (state == 1 && Math.abs(arm.getError().in(Units.Degrees)) < 10) {
            take.runOuttake(() -> 1).schedule();
            state++;
            timer.reset();
        } else if (state == 2 && timer.hasElapsed(0.5)) {
            arm.setTargetStay(Arm.State.HOLDING).schedule();
            state++;
            timer.reset();
        } else if (state == 3 && Math.abs(arm.getError().in(Units.Degrees)) < 30) {
            take.runTakeOnce(0).schedule();
            elevator.setTargetOnly(Elevator.State.L0).schedule();
            state++;
        }
    }
}
