package frc.lib.util;

import edu.wpi.first.wpilibj.Timer;

import java.util.function.BooleanSupplier;

public class TimedCondition implements BooleanSupplier {
    private final BooleanSupplier condition;
    private final double secondsAfter;
    private final Timer timer = new Timer();
    private boolean lastCondition;

    public TimedCondition(BooleanSupplier condition, double secondsAfter) {
        this.condition = condition;
        this.secondsAfter = secondsAfter;
    }
    public TimedCondition(BooleanSupplier condition, double secondsAfter, boolean risingEdge) {
        this(risingEdge ? condition : () -> !condition.getAsBoolean(), secondsAfter);
    }

    @Override
    public boolean getAsBoolean() {
        boolean value = condition.getAsBoolean();
        if (value && !lastCondition)
            timer.reset();
        lastCondition = value;
        return value && timer.hasElapsed(secondsAfter);
    }
}
