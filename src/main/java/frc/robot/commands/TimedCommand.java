package frc.robot.commands;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class TimedCommand extends FunctionalCommand {
    private long startTime;
    private final long waitTime;
    
    public TimedCommand(Runnable toRun, Runnable endRun, long waitTimeMS, Subsystem... requirements) {
        super(() -> {}, toRun, (Consumer<Boolean>) endRun, () -> false, requirements);
        this.waitTime = waitTimeMS;
    }


    public TimedCommand(Command command, long waitTimeMS) {
        super(command::initialize, command::execute, command::end, command::isFinished, command.getRequirements().toArray(new Subsystem[]{}));
        this.waitTime = waitTimeMS;
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
        super.initialize();
    }

    @Override
    public boolean isFinished() {
        return super.isFinished() || System.currentTimeMillis() - startTime > waitTime;
    }
}
