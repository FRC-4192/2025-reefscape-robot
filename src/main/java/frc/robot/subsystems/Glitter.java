package frc.robot.subsystems;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Glitter extends SubsystemBase {
    private PWM led;

    private int lastLed = 1065;
    private boolean ready;

    private State state = State.INTAKENOTREADY;

    public static enum State{
        FIELDCENTRIC(1905),
        ROBOTCENTRIC(1785),
        INTAKEREADY(1885),
        INTAKENOTREADY(1005);
        private final int ledNum;
        private State(int num) {
            this.ledNum = num;
        }

        public int num() {
            return ledNum;
        }
    }

    public Glitter() {
        led = new PWM(19);
        led.setPulseTimeMicroseconds(lastLed);
        ready=false;
    }
    
    public Command increasePWM() {
        return runOnce(() -> led.setPulseTimeMicroseconds(lastLed += 10));
    }
    public Command dereasePWM() {
        return runOnce(() -> led.setPulseTimeMicroseconds(lastLed -= 10));
    }

    public Command setPWM(State state) {
        return runOnce(() -> this.state=state);
    }
    public Command setPWM(int value) {
        return runOnce(() -> setPWMRaw(value));
    }
    
    public void setPWMRaw(int value){
        led.setPulseTimeMicroseconds(value);
        
    }
    public void setPWMRaw(){
        led.setPulseTimeMicroseconds(state.num());
    }
}
