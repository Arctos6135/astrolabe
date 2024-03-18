package astrolabe.follow;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class AstrolabeAuto extends Command {
    private final Command inner;

    public AstrolabeAuto(Command inner) {
        this.inner = inner;

        for (Subsystem subsystem : inner.getRequirements()) {
            addRequirements(subsystem);
        }
    }

    public AstrolabeAuto(String autoName) {
        try {
            String filePath = Filesystem.getDeployDirectory() + "/pathplanner/autos/" + autoName + ".auto";
            inner = null; // TODO: load the auto here

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
