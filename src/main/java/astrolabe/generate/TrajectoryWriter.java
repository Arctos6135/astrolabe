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
    public static JSONObject serializeTrajectory(long pathHash, long astrolabeCommit, Trajectory trajectory) {
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

        JSONObject json = new JSONObject();
        json.put("states", jsonStates);
        json.put("pathHash", pathHash);
        json.put("astrolabeCommit", astrolabeCommit);

        return json;
    }

    public static void writeTrajectory(long pathHash, long astrolabeCommit, Trajectory trajectory, File file) throws IOException {
        JSONObject array = serializeTrajectory(pathHash, astrolabeCommit, trajectory);
        BufferedWriter writer = new BufferedWriter(new FileWriter(file));
        writer.write(array.toJSONString());
        writer.close();
    }

    public static void writeTrajectory(long pathHash, long astrolabeCommit, Trajectory trajectory, String filePath) throws IOException {
        File file = new File(filePath);

        file.getParentFile().mkdirs();

        file.createNewFile();
        writeTrajectory(pathHash, astrolabeCommit, trajectory, file);
    }
}
