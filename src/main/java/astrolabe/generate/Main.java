package astrolabe.generate;

import java.io.File;

public class Main {
    public static void main(String[] args) throws Exception {
        String regenerate = System.getenv("GENERATE_ALL");
        String root = System.getenv("DEPLOY_RELATIVE");

        System.out.println("Generating with root directory " + root);

        String deployDirectory = root + "src/main/deploy/";

        int total = 0;

        System.out.printf("Absolute path of deploy directory is %s\n", new File(deployDirectory).getAbsolutePath());
        System.out.printf("Checking for paths in %s\n", new File(deployDirectory + "pathplanner/paths").getAbsolutePath());

        for (File file : new File(deployDirectory + "pathplanner/paths").listFiles()) {
            if (!file.getName().matches(".*\\.path")) continue;
            String pathName = file.getName().split("\\.")[0];
            System.out.printf("Found file %s with name %s\n", file.getAbsolutePath(), pathName);

            if (regenerate == null) {
                if (UpdateChecker.checkUpdates(pathName, deployDirectory)) total++;
            } else {
                total++;
                UpdateChecker.update(UpdateChecker.hashPath(pathName, deployDirectory), UpdateChecker.currentAstrolabeCommit(), pathName, deployDirectory);
            }
        }

        System.out.printf("Generated a total of %s trajectories\n", total);
    }
}
