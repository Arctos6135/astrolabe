package astrolabe.follow;
  
import java.io.FileReader;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.Iterator; 
import java.util.Map;
import java.util.Objects;

import org.json.simple.JSONArray; 
import org.json.simple.JSONObject; 
import org.json.simple.parser.*;
import org.json.simple.parser.JSONParser;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class PathParser {
    public static AstrolabePath loadPath(String pathName) throws Exception {
        JSONObject json = (JSONObject) new JSONParser().parse(new FileReader(pathName));
        System.out.println(json.toString());

        ArrayDeque<Pose2d> anchors = new ArrayDeque<>();

        for (Object waypoint : (JSONArray) json.get("waypoints")) {
            anchors.add(parseWaypoint((JSONObject) waypoint));
        }

        Pose2d start = anchors.pollFirst();
        Pose2d end = anchors.pollLast();
        Objects.requireNonNull(start);
        Objects.requireNonNull(end);
        boolean reversed = (boolean) json.get("reversed");

        ArrayList<Translation2d> waypoints = new ArrayList<>();

        for (Pose2d pose : anchors) {
            waypoints.add(pose.getTranslation());
            System.out.println(pose.toString());
        }

        if (reversed) {
            start = new Pose2d(start.getTranslation(), start.getRotation().plus(Rotation2d.fromDegrees(180)));
            end = new Pose2d(end.getTranslation(), end.getRotation().plus(Rotation2d.fromDegrees(180)));
        }

        return new AstrolabePath(start, waypoints, end, reversed);
    }

    public static Pose2d parseWaypoint(JSONObject waypoint) {
        Translation2d anchor = parseXY((JSONObject) waypoint.get("anchor"));
        Translation2d delta;

        if (waypoint.get("nextControl") instanceof JSONObject nextControlJson) {
            Translation2d nextControl = parseXY(nextControlJson);
            delta = nextControl.minus(anchor);
        } else if (waypoint.get("prevControl") instanceof JSONObject prevControlJson) {
            Translation2d prevControl = parseXY(prevControlJson);
            delta = anchor.minus(prevControl);
        } else {
            throw new RuntimeException("missing either previous or next control point");
        }

        Rotation2d angle = delta.getAngle();

        return new Pose2d(anchor, angle);
    }

    private static Translation2d parseXY(JSONObject xy) {
        double x = (Double) xy.get("x");
        double y = (Double) xy.get("y");
        return new Translation2d(
            x,
            y
        );
    }
}