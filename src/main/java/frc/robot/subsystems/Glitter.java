package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Glitter extends SubsystemBase {
    private final PWM led;
    private State state = State.FIELD_CENTRIC;

    public enum State {
//        FIELDCENTRIC(1905),
//        ROBOTCENTRIC(1785),
        FIELD_CENTRIC(1315),
        ROBOT_CENTRIC(1305),
        FIELD_SPEED(1305),
        ROBOT_SPEED(1135),
        INTAKEREADY(1885),
        INTAKENOTREADY(1005);

        public final int code;

        State(int ledCode) {
            this.code = ledCode;
        }
    }

    public Glitter() {
        led = new PWM(14);
    }

//    public Command increasePWM() {
//        return runOnce(() -> led.setPulseTimeMicroseconds(lastLed += 10));
//    }
//
//    public Command dereasePWM() {
//        return runOnce(() -> led.setPulseTimeMicroseconds(lastLed -= 10));
//    }

    public State getState() {
        return state;
    }

    public void setState(State state) {
        this.state = state;
    }

    public boolean isFieldCentric() {
        return getState() == State.FIELD_CENTRIC || getState() == State.FIELD_SPEED;
    }
    public boolean isSpeed() {
        return getState() == State.FIELD_SPEED || getState() == State.ROBOT_SPEED;
    }

    public void toggleDrivePov() {
        switch (getState()) {
            case FIELD_CENTRIC -> setState(State.ROBOT_CENTRIC);
            case ROBOT_CENTRIC -> setState(State.FIELD_CENTRIC);
            case FIELD_SPEED -> setState(State.ROBOT_SPEED);
            case ROBOT_SPEED -> setState(State.FIELD_SPEED);
            default -> throw new IllegalStateException("Unexpected value: " + getState());
        };
    }

    public void toggleDriveSpeed() {
        switch (getState()) {
            case FIELD_CENTRIC -> setState(State.FIELD_SPEED);
            case ROBOT_CENTRIC -> setState(State.ROBOT_SPEED);
            case FIELD_SPEED -> setState(State.FIELD_CENTRIC);
            case ROBOT_SPEED -> setState(State.ROBOT_CENTRIC);
            default -> throw new IllegalStateException("Unexpected value: " + getState());
        };
    }

//    public Command setDriveSpeed(boolean normal) {
//        return normal ==
//    }

    public Command setPWM(int value) {
        return runOnce(() -> setPWMRaw(value));
    }

    public void setPWMRaw(int value) {
        led.setPulseTimeMicroseconds(value);

    }

    @Override
    public void periodic() {
        led.setPulseTimeMicroseconds(state.code);
        SmartDashboard.putString("glitter state", state.name());
    }

}
