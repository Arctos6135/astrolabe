package astrolabe.generate.spline;

import astrolabe.follow.AstrolabePath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Heuristic {
    public record XYSpline(Spline x, Spline y) {}
    public static XYSpline fromPath(AstrolabePath path) {
        // dy/dx = tan theta
        double dx = 1;
        double dy = path.startPose().getRotation().getTan();
        double ddx = 0;
        double ddy = 0;

        Pose2d lastPose = path.startPose();

        Segment[] xs = new Segment[path.waypoints().size() + 1];
        Segment[] ys = new Segment[path.waypoints().size() + 1];

        for (int i = 0; i < path.waypoints().size(); i++) {
            Translation2d currentPose = path.waypoints().get(i);
        }

        return null;
    }
}
