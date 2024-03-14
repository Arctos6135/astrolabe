package astrolabe;

import java.util.function.Consumer;

import edu.wpi.first.math.geometry.Pose2d;

public class AstrolabeLogger {
    public static Consumer<Pose2d> targetPoseLogger;
    public static Consumer<Integer> stateLogger;
}
