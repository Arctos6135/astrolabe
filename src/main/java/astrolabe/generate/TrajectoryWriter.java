package astrolabe.generate;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Filesystem;

public class TrajectoryWriter {
    public static JSONArray serializeTrajectory(Trajectory trajectory) {
        JSONArray jsonStates = new JSONArray();
        for (State state : trajectory.getStates()) {
            Objects.requireNonNull(state.accelerationMetersPerSecondSq);
            Objects.requireNonNull(state.curvatureRadPerMeter);
            Objects.requireNonNull(state.velocityMetersPerSecond);
            Objects.requireNonNull(state.timeSeconds);
            Objects.requireNonNull(state.poseMeters);
            JSONObject jsonState = new JSONObject(Map.of(
                "x", state.poseMeters.getX(),
                "y", state.poseMeters.getY(),
                "theta", state.poseMeters.getRotation().getRadians(),
                "acceleration", state.accelerationMetersPerSecondSq,
                "curvature", state.curvatureRadPerMeter,
                "time", state.timeSeconds,
                "velocity", state.velocityMetersPerSecond
            ));
            jsonStates.add((Object) jsonState);
        }

        return jsonStates;
    }

    public static void writeTrajectory(Trajectory trajectory, File file) throws IOException {
        JSONArray array = serializeTrajectory(trajectory);
        BufferedWriter writer = new BufferedWriter(new FileWriter(file));
        writer.write(array.toJSONString());
        writer.close();
    }

    public static void writeTrajectory(Trajectory trajectory, String filePath) throws IOException {
        File file = new File(filePath);
        System.out.println(file.getAbsolutePath());

        file.getParentFile().mkdirs();

        file.createNewFile();
        writeTrajectory(trajectory, file);
    }
}
