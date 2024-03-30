package astrolabe.generate;

import java.util.Arrays;
import java.util.List;

import astrolabe.follow.AstrolabePath;
import astrolabe.generate.spline.Heuristic;
import astrolabe.generate.spline.Profiler;
import astrolabe.generate.spline.Heuristic.XYSpline;
import edu.wpi.first.math.trajectory.Trajectory;

public class Optimizer {
    private List<Double> ddxs;
    private List<Double> ddys;
    private final AstrolabePath path;

    public Optimizer(AstrolabePath path) {
        this.path = path;


        Double[] ddxs = new Double[path.waypoints().size() + 2];
        Double[] ddys = new Double[path.waypoints().size() + 2];

        for (int i = 0; i < ddys.length; i++) {
            ddxs[i] = 0.0;
            ddys[i] = 0.0;
        }
        
        this.ddxs = Arrays.asList(ddxs);
        this.ddys = Arrays.asList(ddxs);
    }

    public double evaluate() {
        XYSpline spline = Heuristic.fromPathForward(path, ddxs, ddys);
        return Profiler.fromSpline(spline.x(), spline.y(), 3, 1.5, path.reversed(), true).getTotalTimeSeconds();
    }

    public void step() {
        double normal = evaluate();

        for (int i = 0; i < ddxs.size(); i++) {
            double derivative;

            double best = evaluate();

            do {
                normal = evaluate();
                if (normal > best) {
                    best = normal;
                } else {
                    break;
                }
                ddxs.set(i, ddxs.get(i) + 0.0001);
                derivative = (evaluate() - normal) / 0.0001;
                ddxs.set(i, ddxs.get(i) - 0.0001);

                if (derivative > 0) {
                    ddxs.set(i, ddxs.get(i) + 0.0001);
                } else {
                    ddxs.set(i, ddxs.get(i) - 0.0001);
                }
            } while (true);

            best = evaluate();
            do {
                normal = evaluate();
                if (normal > best) {
                    best = normal;
                } else {
                    break;
                }
                ddys.set(i, ddys.get(i) + 0.0001);
                derivative = (evaluate() - normal) / 0.0001;
                ddys.set(i, ddys.get(i) - 0.0001);

                if (derivative > 0) {
                    ddys.set(i, ddys.get(i) + 0.0001);
                } else {
                    ddys.set(i, ddys.get(i) - 0.0001);
                }
            } while (true);
        }
    }

    public Trajectory run() {
        for (int i = 0; i < 100; i++) {
            step();
        }

        XYSpline spline = Heuristic.fromPathForward(path, ddxs, ddys);

        return Profiler.fromSpline(spline.x(), spline.y(), 3.5, 2.5, path.reversed());
    }
}
