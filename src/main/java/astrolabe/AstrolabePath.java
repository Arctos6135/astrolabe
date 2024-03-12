package astrolabe;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

public record AstrolabePath(
    Pose2d startPose,
    List<Translation2d> waypoints,
    Pose2d endPose,
    boolean reversed
) {
    public Trajectory generateTrajectory(GlobalConfig globalConfig) {
        var config = globalConfig.trajectoryConfig(reversed);
        return TrajectoryGenerator.generateTrajectory(startPose, waypoints, endPose, config);
    }
}
