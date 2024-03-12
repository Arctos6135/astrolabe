package astrolabe;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public record AstrolabePath(
    Pose2d startPose,
    List<Translation2d> waypoints,
    Pose2d endPose,
    boolean reversed
) {}
