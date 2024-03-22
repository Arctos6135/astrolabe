package astrolabe.follow;

import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;

public class TrajectoryParser {
    public static Trajectory ParseTrajectory(String pathName) throws Exception {
        JSONObject json = (JSONObject) new JSONParser().parse(new FileReader(pathName));
        JSONArray statesMap = (JSONArray) json.get("states");
        List<State> states = new ArrayList<>();

        for (Object object : statesMap) {
            JSONObject state = (JSONObject) object;
            double time = (double) state.get("time");
            double velocity = (double) state.get("velocity");
            double acceleration = (double) state.get("acceleration");
            Pose2d pose = new Pose2d((double) state.get("x"), (double) state.get("y"), new Rotation2d((double) state.get("theta")));
            double curvature = (double) state.get("curvature");
            states.add(new State(time, velocity, acceleration, pose, curvature));
        }
        
        return new Trajectory(states);
    }
}
