package frc.lib.util;

public class LowPassFilter2 {
    private final double a;
    private double lastValue;

    /**
     * @param a between 0 and 1
     */
    public LowPassFilter2(double a) {
        this.a = a;
    }

    public double getA() {
        return a;
    }

    public double update(double value) {
        return lastValue = a * value + (1 - a) * lastValue;
    }
}
