package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Blinkin {

    Spark blinkin;

	public Blinkin() {
		blinkin = new Spark(19);
	}
	public void intakeReady() {
		blinkin.set(0.77);
	}

	public void intakeNotReady() {
		blinkin.set(.59); 
	}
}
