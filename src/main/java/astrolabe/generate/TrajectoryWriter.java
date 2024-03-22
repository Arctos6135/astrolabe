package astrolabe.generate;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Filesystem;

public class TrajectoryWriter {
    public static JSONArray serializeTrajectory(Trajectory trajectory) {
        ArrayList<Object> jsonStates = new ArrayList<>();
        for (State state : trajectory.getStates()) {
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

        return (JSONArray) (List<Object>) jsonStates;
    }

    public static void writeTrajectory(Trajectory trajectory, File file) throws IOException {
        JSONArray array = serializeTrajectory(trajectory);
        BufferedWriter writer = new BufferedWriter(new FileWriter(file));
        writer.write(array.toJSONString());
        writer.close();
    }

    public static void writeTrajectory(Trajectory trajectory, String pathName) throws IOException {
        String filePath = Filesystem.getDeployDirectory() + "/pathplanner/trajectories/" + pathName + ".traj";

        writeTrajectory(trajectory, new File(filePath));
    }
}
