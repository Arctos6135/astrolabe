package astrolabe.generate.spline;

import java.util.ArrayDeque;
import java.util.ArrayList;

import astrolabe.follow.AstrolabePath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Heuristic {
    public record XYSpline(Spline x, Spline y) {}
    private record SplineState(
        double x, double dx, double ddx,
        double y, double dy, double ddy
    ) {}

    public static XYSpline fromPath(AstrolabePath path) {
        if (path.reversed()) {
            return fromPathReversed(path);
        } else {
            return fromPathForward(path);
        }
    }

    public static XYSpline fromPathForward(AstrolabePath path) {
        // dy/dx = tan theta
        double dx = 1;
        double dy = path.startPose().getRotation().getTan();
        double ddx = 0;
        double ddy = 0;

        double dxEnd = 1;
        double dyEnd = path.endPose().getRotation().getTan();
        double ddxEnd = 0;
        double ddyEnd = 0;

        ArrayList<Translation2d> positions = new ArrayList<>();
        positions.add(path.startPose().getTranslation());

        for (int i = 0; i < path.waypoints().size(); i++) {
            Translation2d current = path.waypoints().get(i);
            positions.add(current);
        }

        positions.add(path.endPose().getTranslation());

        ArrayList<Rotation2d> angles = new ArrayList<>();

        for (int i = 1; i < positions.size() - 1; i++) {
            Translation2d start = positions.get(i - 1);
            Translation2d mid = positions.get(i);
            Translation2d end = positions.get(i + 1);

            Translation2d toMid = mid.minus(start);
            Translation2d fromMid = end.minus(mid);

            Rotation2d angle = toMid.getAngle().plus(fromMid.getAngle()).times(0.5);
            angles.add(angle);
        }

        ArrayList<SplineState> states = new ArrayList<>();
        states.add(new SplineState(path.startPose().getX(), dx, ddx, path.startPose().getY(), dy, ddy));

        for (int i = 0; i < angles.size(); i++) {
            states.add(new SplineState(
                path.waypoints().get(i).getX(), angles.get(i).getCos(), 0, 
                path.waypoints().get(i).getY(), angles.get(i).getSin(), 0
            ));
        }

        states.add(new SplineState(path.endPose().getX(), dxEnd, ddxEnd, path.endPose().getY(), dyEnd, ddyEnd));

        Segment[] xs = new Segment[path.waypoints().size() + 1];
        Segment[] ys = new Segment[path.waypoints().size() + 1];

        for (int i = 0; i < states.size() - 1; i++) {
            SplineState start = states.get(i);
            SplineState end = states.get(i + 1);

            xs[i] = Segment.fromTangents(start.x, end.x, start.dx, end.dx, start.ddx, end.ddx);
            ys[i] = Segment.fromTangents(start.y, end.y, start.dy, end.dy, start.ddy, end.ddy);
        }

        return new XYSpline(new Spline(xs), new Spline(ys));
    }

    public static XYSpline fromPathReversed(AstrolabePath path) {
        ArrayList<Translation2d> waypoints = new ArrayList<>();
        for (int index = 0; index < path.waypoints().size(); index++) {
            waypoints.add(path.waypoints().get(path.waypoints().size() - 1 - index));
        }
        AstrolabePath reversed = new AstrolabePath(path.endPose(), waypoints, path.startPose(), false);
        return fromPathForward(reversed);
    }
}
