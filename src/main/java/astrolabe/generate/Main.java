package astrolabe.generate;

import java.util.Arrays;
import java.util.Scanner;

import astrolabe.follow.AstrolabePath;
import astrolabe.follow.PathParser;
import astrolabe.generate.spline.Heuristic;
import astrolabe.generate.spline.Profiler;
import astrolabe.generate.spline.Heuristic.XYSpline;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Filesystem;

public class Main {
    public static void main(String[] args) throws Exception {
        String name = System.getenv("TO_DEPLOY");
        String root = System.getenv("DEPLOY_RELATIVE");

        //String pathName = "./src/main/deploy" + "/pathplanner/paths/" + name + ".path";
        String pathName = root + "./src/main/deploy/pathplanner/paths/" + name + ".path";
        String outputPath = root + "./src/main/deploy/pathplanner/trajectories/" + name + ".traj";

        AstrolabePath path = PathParser.loadPath(pathName);
        System.out.println(path.toString());
        XYSpline spline = Heuristic.fromPath(path);
        System.out.println(spline.toString());
        System.out.println(spline.x());
        Trajectory trajectory = Profiler.fromSpline(spline.x(), spline.y(), 2.5, 1, path.reversed());
        TrajectoryWriter.writeTrajectory(trajectory, outputPath);
    }
}
