package astrolabe.generate.spline;

public record Segment(
    double[] coefficients
) {
    /**
     * Evaluate the spline at a given point
     * @param t the point in the range [0, 1] to evaluate the spline at
     * @return
     */
    public double evaluate(double t) {
        double inverse = 1 - t;
        double total = 0;

        for (int i = 0; i < coefficients.length; i++) {
            double term = Math.pow(t, i) * Math.pow(inverse, coefficients.length - 1 - i) * coefficients[i];
            total += term;
        }

        return total;
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
}