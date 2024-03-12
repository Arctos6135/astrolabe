package astrolabe;

import edu.wpi.first.wpilibj2.command.Command;

public class AstrolabeAuto extends Command {
    private final Command inner;

    public AstrolabeAuto(AstrolabePath path) {
        inner = AutoBuilder.buildPathFollowingCommand(path);
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
