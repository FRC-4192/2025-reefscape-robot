package frc.lib.util;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotController;

public class JerkFilter extends TrapezoidProfile {
    private State lastState = new State();
    private State lastGoal = new State();
    private long lastTime = 0;

    public JerkFilter(Constraints constraints) {
        super(constraints);
    }

    /**
     *
     * @param setpoint The current position reading
     * @param timeUs Time since last update
     * @return new position
     */
    public double calculate(double setpoint, long timeUs) {
        double timeDelta = (timeUs - lastTime) * 1e-6;
        lastTime = timeUs;
        return (lastState = calculate(
                timeDelta,
                lastState,
                lastGoal = new State(setpoint, (setpoint - lastGoal.position) / timeDelta)
        )).position;
    }

    /**
     * @param position The current position reading
     * @return The current filtered position value
     */
    public double calculate(double position) {
        return calculate(position, RobotController.getTime());
    }
}
