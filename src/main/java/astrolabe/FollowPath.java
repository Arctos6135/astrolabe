package astrolabe;

import javax.management.RuntimeErrorException;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class FollowPath extends Command {
    private final Command inner;

    public FollowPath(AstrolabePath path) {
        inner = AutoBuilder.buildPathFollowingCommand(path);

        for (Subsystem subsystem : inner.getRequirements()) {
            addRequirements(subsystem);
        }
    }

    public FollowPath(String pathName) {
        try {
            String filePath = Filesystem.getDeployDirectory() + "/pathplanner/paths/" + pathName + ".path";
            inner = AutoBuilder.buildPathFollowingCommand(PathParser.loadPath(filePath));

            for (Subsystem subsystem : inner.getRequirements()) {
                addRequirements(subsystem);
            }
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void initialize() {
        inner.initialize();
    }

    @Override
    public void execute() {
        inner.execute();
    }

    @Override
    public boolean isFinished() {
        return inner.isFinished();
    }

    @Override
    public void end(boolean i) {
        inner.end(i);
    }
}
