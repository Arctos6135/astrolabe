package astrolabe.generate;

import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.nio.CharBuffer;
import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import astrolabe.follow.AstrolabePath;
import astrolabe.follow.PathParser;
import astrolabe.generate.spline.Heuristic;
import astrolabe.generate.spline.Heuristic.XYSpline;
import astrolabe.generate.spline.Profiler;
import edu.wpi.first.math.trajectory.Trajectory;
import net.jpountz.xxhash.XXHash64;
import net.jpountz.xxhash.XXHashFactory;

public class UpdateChecker {
    public static boolean checkUpdates(String name, String deployDirectory) throws Exception {
        long hash = hashPath(name, deployDirectory);
        long astrolabeCommit = currentAstrolabeCommit();

        if (!new File(deployDirectory + "pathplanner/trajectories/" + name + ".traj").exists()) {
            update(hash, astrolabeCommit, name, deployDirectory);
            return true;
        }

        FileReader trajReader = new FileReader(deployDirectory + "pathplanner/trajectories/" + name + ".traj");
        JSONObject json = (JSONObject) new JSONParser().parse(trajReader);

        long oldHash = (long) json.get("pathHash");
        long oldAstrolabeCommit = (long) json.get("astrolabeCommit");

        trajReader.close();

        if (hash != oldHash || astrolabeCommit != oldAstrolabeCommit) {
            update(hash, astrolabeCommit, name, deployDirectory);
            return true;
        } else {
            return false;
        }
    }

    public static void update(long hash, long astrolabeCommit, String name, String deployDirectory) throws Exception {
        System.out.println("Generating " + name);
        String pathName = deployDirectory + "pathplanner/paths/" + name + ".path";
        String outputPath = deployDirectory + "pathplanner/trajectories/" + name + ".traj";

        AstrolabePath path = PathParser.loadPath(pathName);
        XYSpline spline = Heuristic.fromPath(path);
        Trajectory trajectory = Profiler.fromSpline(spline.x(), spline.y(), 2.5, 2, path.reversed());
        TrajectoryWriter.writeTrajectory(hash, astrolabeCommit, trajectory, outputPath);

        System.out.printf("\t%s states and %s seconds\n", trajectory.getStates().size(), trajectory.getTotalTimeSeconds());
    }

    public static long hashPath(String name, String deployDirectory) throws Exception {
        XXHash64 hasher = XXHashFactory.nativeInstance().hash64();

        FileReader reader = new FileReader(deployDirectory + "pathplanner/paths/" + name + ".path");

        ArrayList<Character> characters = new ArrayList<>();

        while (true) {
            int c = reader.read();
            if (c == -1) break;
            characters.add((char) c);
        }

        byte[] bytes = new byte[characters.size()];

        for (int i = 0; i < characters.size(); i++) {
            bytes[i] = (byte) characters.get(i).charValue();
        }

        reader.close();

        return hasher.hash(bytes, 0, bytes.length, 0);
    }

    public static long currentAstrolabeCommit() throws Exception {
        Runtime runtime = Runtime.getRuntime();
        Process process = runtime.exec("git rev-parse HEAD");
        process.waitFor(1, TimeUnit.SECONDS);
        byte[] bytes = process.getInputStream().readAllBytes();

        XXHash64 hasher = XXHashFactory.nativeInstance().hash64();

        return hasher.hash(bytes, 0, bytes.length, 0);
    }
}
