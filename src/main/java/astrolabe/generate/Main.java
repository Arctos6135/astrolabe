package astrolabe.generate;

import java.util.Scanner;

import astrolabe.follow.AstrolabePath;
import astrolabe.follow.PathParser;
import astrolabe.generate.spline.Heuristic;
import astrolabe.generate.spline.Profiler;
import astrolabe.generate.spline.Heuristic.XYSpline;
import edu.wpi.first.math.trajectory.Trajectory;

public class Main {
    public static void main(String[] args) throws Exception {
        String pathName = new Scanner(System.in).nextLine();
        AstrolabePath path = PathParser.loadPath(pathName);
        XYSpline spline = Heuristic.fromPath(path);
        Trajectory trajectory = Profiler.fromSpline(spline.x(), spline.y(), 3, 3);
    }
}
