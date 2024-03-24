package astrolabe.generate;

import java.io.File;

public class Main {
    public static void main(String[] args) throws Exception {
        String name = System.getenv("GENERATE_ALL");
        String root = System.getenv("DEPLOY_RELATIVE");

        System.out.println("Generating with root directory " + root);

        String deployDirectory = root + "src/main/deploy/";

        int total = 0;

        for (File file : new File(deployDirectory + "pathplanner/paths").listFiles()) {
            String pathName = file.getName().split("\\.")[0];

            if (name == null) {
                if (UpdateChecker.checkUpdates(pathName, deployDirectory)) total++;
            } else {
                total++;
                UpdateChecker.update(UpdateChecker.hashPath(pathName, deployDirectory), pathName, deployDirectory);
            }
        }

        System.out.printf("Generated a total of %s trajectories\n", total);
    }
}
