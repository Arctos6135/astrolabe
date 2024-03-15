package astrolabe;

import java.util.function.Consumer;
import java.util.function.DoubleConsumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;

public class AstrolabeLogger {
    public static Consumer<Pose2d> targetPoseLogger;
    public static Consumer<Integer> stateLogger;
    public static Consumer<Trajectory> trajectoryLogger;
    public static DoubleConsumer angleErrorDegreesLogger;
    public static DoubleConsumer distanceErrorLogger;
}
