package frc.lib.util;

public class ExponentialLowPassFilter {
    public double coeff;
    public double factor;

    private Double lastVal;
    private Double rawVal;

    public ExponentialLowPassFilter(double coeff, double factor) {
        this.coeff = coeff;
        this.factor = factor;
    }

    public ExponentialLowPassFilter(double factor) {
        this(0.5, factor);
    }

    public ExponentialLowPassFilter() {
        this(0.5, 1);
    }

    public void setValues(double coeff, double factor) {
        setCoefficient(coeff);
        setFactor(factor);
    }

    public void setCoefficient(double coeff) {
        this.coeff = Math.max(0, Math.min(coeff, 1));
    }

    public void setFactor(double factor) {
        this.factor = factor;
    }

    public double update(double value) {
        rawVal = value;

        if (lastVal == null || Double.isNaN(lastVal)) {
            lastVal = value;
            return value;
        }

        double origLastVal = lastVal;

//        lastVal = (coeff * value) + ((1 - coeff) * lastVal);
        lastVal = lastVal + (Math.pow(Math.abs(value - lastVal), factor) * Math.signum(value - lastVal) * coeff);

        if (value < origLastVal && lastVal < value) lastVal = value;
        if (value > origLastVal && lastVal > value) lastVal = value;

        return lastVal;
    }

    public Double getLastValue() {
        return lastVal;
    }

    public Double getRawValue() {
        return rawVal;
    }
}