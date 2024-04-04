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
        public double maxAcceleration;
        public double minAcceleration;
        public double acceleration;

        @Override
        public String toString() {
            return String.format("ConstrainedState [distance=%.2f, velocity=%.2f, max acceleration=%.2f, min acceleration=%.2f]", distance, velocity, maxAcceleration, minAcceleration);
        }
    }
    private record State(
        Pose2d pose, double curvature
    ) {}
    // private static List<State> distanceProfile(
    //     Segment xSpline,
    //     Segment ySpline,
    //     double arcLength
    // ) {
    //     ArrayList<State> samples = new ArrayList<>();
    //     samples.add(getPoint(xSpline, ySpline, 0));

    //     double lastT = 0;
    //     double lastX = xSpline.sample(0);
    //     double lastY = ySpline.sample(0);

    //     double t;

    //     for (t = 0; t <= 1; t += 0.1) {
    //         double x = xSpline.sample(t);
    //         double y = ySpline.sample(t);
    //         // very sketchy way to find the arc length from the last point
    //         double currentArc = Math.sqrt(Math.pow(lastT - t, 2) + Math.pow(lastX - x, 2) + Math.pow(lastY - y, 2));

    //         if (currentArc >= arcLength) {
    //             samples.add(getPoint(xSpline, ySpline, t));
    //             lastT = t;
    //             lastX = x;
    //             lastY = t;
    //         }
    //     }

    //     return samples;
    // }

    private static List<State> distanceProfile(
        Segment xSpline,
        Segment ySpline,
        double arcLength,
        boolean skipFirst
    ) {
        ArrayList<State> samples = new ArrayList<>();
        samples.add(getPoint(xSpline, ySpline, 0));

        double lastX = xSpline.sample(0);
        double lastY = ySpline.sample(0);

        double t;

        for (t = 0; t <= 1; t += 0.001) {
            double x = xSpline.sample(t);
            double y = ySpline.sample(t);
            // very sketchy way to find the arc length from the last point
            double currentArc = Math.sqrt(Math.pow(lastX - x, 2) + Math.pow(lastY - y, 2));

            if (currentArc >= arcLength) {
                samples.add(getPoint(xSpline, ySpline, t));
                lastX = x;
                lastY = y;
            }
        }

        if (skipFirst) {
            samples.remove(0);
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

        double epsilon = 1e-6;

        {
            Pose2d previous = states.get(0).pose;
            for (State state : states) {
                ConstrainedState inner = new ConstrainedState();
                inner.distance = state.pose.getTranslation().getDistance(previous.getTranslation());
                inner.state = state;
                inner.velocity = maxVelocity;
                inner.maxAcceleration = maxAcceleration;
                inner.minAcceleration = -maxAcceleration;

                previous = inner.state.pose;
                constrained.add(inner);
            }
        }

        //System.out.println("After initialization:");
        for (ConstrainedState constrainedState : constrained) {
            //System.out.println(constrainedState.toString());
        }

        {
            ConstrainedState previous = constrained.get(0);
            previous.velocity = startVelocity;

            for (int i = 1; i < constrained.size(); i++) {
                ConstrainedState current = constrained.get(i);

                current.velocity = Math.min(
                    current.velocity,
                    Math.sqrt(previous.velocity * previous.velocity + 2 * previous.maxAcceleration * current.distance)
                );

                previous = current;
            }
        }

        //System.out.println("After fist pass:");
        for (ConstrainedState constrainedState : constrained) {
            //System.out.println(constrainedState.toString());
        }

        {
            ConstrainedState next = constrained.get(constrained.size() - 1);
            next.velocity = endVelocity;
            next.minAcceleration = -maxAcceleration;
            next.maxAcceleration = maxAcceleration;

            for (int i = constrained.size() - 2; i >= 0; i--) {
                ConstrainedState current = constrained.get(i);

                current.velocity = Math.min(
                    current.velocity, 
                    Math.sqrt(next.velocity * next.velocity - 2 * next.minAcceleration * next.distance)
                );

                next = current;
            }
        }

        //System.out.println("After second pass:");
        for (ConstrainedState constrainedState : constrained) {
            //System.out.println(constrainedState.toString());
        }

        double time = 0;

        for (int i = 0; i < constrained.size() - 1; i++) {
            ConstrainedState current = constrained.get(i);
            ConstrainedState next = constrained.get(i + 1);
            if (Math.abs(next.distance) < epsilon) {
                throw new RuntimeException("Unacceptable distance from previous state in state " + next.toString() + " at index " + i);
            }
            current.time = time;

            double acceleration = (next.velocity * next.velocity - current.velocity * current.velocity) / (2 * next.distance);

            double dt = 0;

            if (Math.abs(acceleration) > epsilon) {
                dt = (next.velocity - current.velocity) / acceleration;
            } else if (Math.abs(current.velocity) > epsilon) {
                dt = next.distance / current.velocity;
            } else {
                throw new RuntimeException("Both acceleration and velocity are 0 at state " + current.toString() + " at index " + i);
            }

            time += dt;

            current.acceleration = acceleration;
        }

        ConstrainedState last = constrained.get(constrained.size() - 1);
        last.time = time;
        last.acceleration = 0;

        List<Trajectory.State> trajectory = new ArrayList<>();

        for (ConstrainedState state : constrained) {
            trajectory.add(new Trajectory.State(state.time, state.velocity, state.acceleration, state.state.pose, state.state.curvature));
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
        return fromSpline(x, y, maxVelocity, maxAcceleration, reversed, false);
    }

    public static Trajectory fromSpline(
        Spline x,
        Spline y,
        double maxVelocity,
        double maxAcceleration,
        boolean reversed,
        boolean fastMode
    ) {
        if (reversed) {
            return reverseTrajectory(fromSplineForward(x, y, maxVelocity, maxAcceleration, fastMode));
        } else {
            return fromSplineForward(x, y, maxVelocity, maxAcceleration, fastMode);
        }
    }

    public static Trajectory fromSplineForward(
        Spline x,
        Spline y,
        double maxVelocity,
        double maxAcceleration,
        boolean fastMode
    ) {
        ArrayList<State> states = new ArrayList<>();

        for (int i = 0; i < x.segments().length; i++) {
            states.addAll(distanceProfile(x.segments()[i], y.segments()[i], fastMode ? 0.1 : 0.03, i != 0));
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
            states.add(new Trajectory.State(maxTime - old.timeSeconds, -old.velocityMetersPerSecond, old.accelerationMetersPerSecondSq, old.poseMeters, old.curvatureRadPerMeter));
        }

        return new Trajectory(states);
    }
}