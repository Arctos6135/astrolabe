package astrolabe.generate.spline;

import java.util.Arrays;

public record Spline(Segment[] segments) {
    public double maxT() {
        return (double) segments.length;
    }

    public double sample(double t) {
        int segment = (int) t;
        return segments[segment].sample(t - segment);
    }

    public double derivative(double t) {
        int segment = (int) t;
        return segments[segment].derivative(t - segment);
    }

    public double secondDerivative(double t) {
        int segment = (int) t;
        return segments[segment].secondDerivative(t - segment);
    }

    @Override
    public String toString() {
        return Arrays.toString(segments);
    }
}
