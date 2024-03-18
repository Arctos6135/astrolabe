package astrolabe.generate.spline;

import java.util.ArrayList;
import java.util.List;

import javax.swing.plaf.nimbus.State;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;

public class Profiler {
    private static class ConstrainedState {
        public State state;
        public double time;
        public double distance;
        public double velocity;
        public double acceleration;
    }
    private record State(
        Pose2d pose, double curvature
    ) {}
    private static List<State> distanceProfile(
        Segment xSpline,
        Segment ySpline,
        double arcLength
    ) {
        ArrayList<State> samples = new ArrayList<>();
        samples.add(getPoint(xSpline, ySpline, 0));

        double lastT = 0;
        double lastX = xSpline.sample(0);
        double lastY = ySpline.sample(0);

        for (double t = 0; t <= 1; t += 0.001) {
            double x = xSpline.sample(t);
            double y = ySpline.sample(t);
            // very sketchy way to find the arc length from the last point
            double currentArc = Math.sqrt(Math.pow(lastT - t, 2) + Math.pow(lastX - x, 2) + Math.pow(lastY - y, 2));

            if (currentArc >= arcLength) {
                samples.add(getPoint(xSpline, ySpline, t));
                lastT = t;
                lastX = x;
                lastY = t;
            }
        }

        return samples;
    }

    private static State getPoint(
        Segment xSpline,
        Segment ySpline,
        double t
    ) {
        double x = xSpline.sample(t);
        double y = ySpline.sample(t);
        double dx = xSpline.derivative(t);
        double dy = ySpline.derivative(t);
        double ddx = xSpline.secondDerivative(t);
        double ddy = ySpline.secondDerivative(t);
        
        double curvature = (dx * ddy - ddx * dy) / ((dx * dx + dy * dy) * Math.hypot(dx, dy));

        return new State(new Pose2d(x, y, Rotation2d.fromRadians(Math.atan2(dy, dx))), curvature);
    }

    private static Trajectory profileTime(
        List<State> states,
        double maxVelocity,
        double maxAcceleration,
        double startVelocity,
        double endVelocity
    ) {
        ArrayList<ConstrainedState> constrained = new ArrayList<>();

        ConstrainedState previous = new ConstrainedState();
        previous.velocity = startVelocity;
        previous.state = states.get(0);

        // Forward pass
        for (int i = 0; i < constrained.size(); i++) {
            ConstrainedState current = new ConstrainedState();

            current.distance = current.state.pose.getTranslation().getDistance(previous.state.pose.getTranslation());

            // enforce positive acceleration constraints
            current.velocity = Math.min(
                maxVelocity,
                Math.sqrt(
                    previous.velocity * previous.velocity + previous.acceleration * current.distance * 2
                )
            );

            previous = current;
        }

        ConstrainedState next = new ConstrainedState();
        next.velocity = endVelocity;
        next.state = states.get(states.size() - 1);

        // Backward pass
        for (int i = constrained.size() - 1; i >= 0; i--) {
            ConstrainedState current = constrained.get(i);

            current.velocity = Math.min(
                current.velocity,
                Math.sqrt(
                    next.velocity * next.velocity - maxAcceleration * current.distance * 2.0
                )
            );
        }

        List<Trajectory.State> trajectory = new ArrayList<>();
        double time = 0;
        double distance = 0;
        double velocity = 0;

        for (int i = 0; i < constrained.size(); i++) {
            ConstrainedState current = constrained.get(i);

            double ds = current.distance - distance;

            double acceleration = (current.velocity * current.velocity - velocity * velocity) / (ds * 2.0);

            double dt = 0.0;

            if (i > 0) {
                trajectory.get(i - 1).accelerationMetersPerSecondSq = acceleration;

                dt = (current.velocity - velocity) / acceleration;
            }

            velocity = current.velocity;
            distance = current.distance;

            time += dt;

            trajectory.add(new Trajectory.State(
                time,
                velocity,
                acceleration,
                current.state.pose,
                current.state.curvature
            ));
        }

        return new Trajectory(trajectory);
    }

    public static Trajectory fromSpline(
        Spline x,
        Spline y,
        double maxVelocity,
        double maxAcceleration
    ) {
        ArrayList<State> states = new ArrayList<>();

        for (int i = 0; i < x.segments().length; i++) {
            states.addAll(distanceProfile(x.segments()[i], y.segments()[i], 0.03));
        }

        return profileTime(states, maxVelocity, maxAcceleration, 0, 0);
    }
}