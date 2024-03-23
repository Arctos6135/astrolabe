package astrolabe.generate.spline;

import java.util.Arrays;

public record Segment(
    double[] coefficients
) {
    private static final double[] binomialFactors = {1, 5, 10, 10, 5, 1};
    /**
     * Evaluate the spline at a given point
     * @param t the point in the range [0, 1] to evaluate the spline at
     * @return
     */
    public double sample(double t) {
        double inverse = 1 - t;
        double total = 0;

        for (int i = 0; i < coefficients.length; i++) {
            double term = binomialFactors[i] * Math.pow(t, i) * Math.pow(inverse, coefficients.length - 1 - i) * coefficients[i];
            total += term;
        }

        return total;
    }

    public double derivative(double t) {
        // TODO: actually compute the derivative instead of sketchily approximating it.
        return (sample(t + 0.0001) - sample(t)) / 0.0001;
    }

    public double secondDerivative(double t) {
        // TODO: actually compute the derivative instead of sketchily approximating it.
        return (derivative(t + 0.0001) - derivative(t)) / 0.0001;
    }

    public static Segment fromTangents(
        double start,
        double end,
        double startTangent,
        double endTangent,
        double startSecond,
        double endSecond
    ) {
        double p0 = start;
        double p5 = end;

        double p1 = 1.0/5.0 * startTangent + p0;
        double p2 = 1.0/20.0 * startSecond + 2 * p1 - p0;
        
        double p4 = p5 - 1.0/5.0 * endTangent;
        double p3 = 1.0/20.0 * endSecond + 2 * p4 - p5;

        return new Segment(new double[]{
            p0, p1, p2, p3, p4, p5
        });
    }

    @Override
    public String toString() {
        return Arrays.toString(coefficients);
    }
}