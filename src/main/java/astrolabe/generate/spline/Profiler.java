package astrolabe.generate.spline;

import java.util.ArrayList;
import java.util.List;

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

        @Override
        public String toString() {
            return String.format("ConstrainedState [distance=%.2f, velocity=%.2f, acceleration=%.2f]", distance, velocity, acceleration);
        }
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

        double t;

        for (t = 0; t <= 1; t += 0.001) {
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

        System.out.printf("Processed %s samples\n", samples.size());

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
        previous.acceleration = maxAcceleration;
        previous.state = states.get(0);
        constrained.add(previous);

        // Forward pass
        for (int i = 1; i < states.size(); i++) {
            ConstrainedState current = new ConstrainedState();
            current.state = states.get(i);

            current.distance = current.state.pose.getTranslation().getDistance(previous.state.pose.getTranslation());


            // enforce positive acceleration constraints
            current.velocity = Math.min(
                maxVelocity,
                Math.sqrt(
                    previous.velocity * previous.velocity + previous.acceleration * current.distance * 2
                )
            );

            current.acceleration = maxAcceleration;


            previous = current;

            constrained.add(current);
        }

        ConstrainedState next = new ConstrainedState();
        next.velocity = endVelocity;
        next.state = states.get(states.size() - 1);

        System.out.printf("Starting with next %s\n", next);

        // Backward pass
        for (int i = constrained.size() - 1; i >= 0; i--) {
            ConstrainedState current = constrained.get(i);

            current.velocity = Math.min(
                current.velocity,
                Math.sqrt(
                    next.velocity * next.velocity + maxAcceleration * current.distance * 2.0
                )
            );

            next = current;

            System.out.println(current);
        }

        List<Trajectory.State> trajectory = new ArrayList<>();
        double time = 0;
        double distance = 0;
        double velocity = 0;

        for (int i = 0; i < constrained.size(); i++) {
            ConstrainedState current = constrained.get(i);

            double ds = current.distance;

            if (ds == 0.0) {
                continue;
            }

            System.out.printf("Current %s, velocity %s, ds %s\n", current.velocity, velocity, ds);

            double acceleration = (current.velocity * current.velocity - velocity * velocity) / (ds * 2.0);

            //System.out.printf("Calculated acceleration %s\n", acceleration);

            double dt = 0.0;

            if (trajectory.size() != 0) {
                trajectory.get(trajectory.size() - 1).accelerationMetersPerSecondSq = acceleration;

                //dt = (current.velocity - velocity) / acceleration;
                dt = ds / current.velocity;
            }

            if (dt < 0) {
                throw new ArithmeticException();
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
        double maxAcceleration,
        boolean reversed
    ) {
        if (reversed) {
            return reverseTrajectory(fromSplineForward(x, y, maxVelocity, maxAcceleration));
        } else {
            return fromSplineForward(x, y, maxVelocity, maxAcceleration);
        }
    }

    public static Trajectory fromSplineForward(
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

    public static Trajectory reverseTrajectory(
        Trajectory trajectory
    ) {
        ArrayList<Trajectory.State> states = new ArrayList<>();
        double maxTime = trajectory.getTotalTimeSeconds();

        for (int i = 0; i < trajectory.getStates().size(); i++) {
            Trajectory.State old = trajectory.getStates().get(trajectory.getStates().size() - 1 - i);
            states.add(new Trajectory.State(maxTime - old.timeSeconds, -old.velocityMetersPerSecond, -old.accelerationMetersPerSecondSq, old.poseMeters, old.curvatureRadPerMeter));
        }

        return new Trajectory(states);
    }
}