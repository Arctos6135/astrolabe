package astrolabe.generate;

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
        String name = new Scanner(System.in).nextLine();

        String pathName = "./src/main/deploy" + "/pathplanner/paths" + name + ".path";

        AstrolabePath path = PathParser.loadPath(pathName);
        XYSpline spline = Heuristic.fromPath(path);
        Trajectory trajectory = Profiler.fromSpline(spline.x(), spline.y(), 3, 3);
        TrajectoryWriter.writeTrajectory(trajectory, pathName);
    }
}
