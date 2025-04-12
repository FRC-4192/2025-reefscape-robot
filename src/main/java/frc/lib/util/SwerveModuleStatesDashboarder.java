package frc.lib.util;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModuleStatesDashboarder {
    public double[] states = new double[8];
    
    public double[] update(SwerveModuleState[] states) {
        for (int i = 0; i < 4; i ++) {
            this.states[2*i] = states[i].angle.getDegrees();
            this.states[2*i+1] = states[i].speedMetersPerSecond;
        }
        return this.states;
    }
}
