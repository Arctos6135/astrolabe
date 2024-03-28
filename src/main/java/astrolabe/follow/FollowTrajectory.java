package astrolabe.follow;

import javax.management.RuntimeErrorException;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.math.trajectory.Trajectory;


public class FollowTrajectory extends Command {
    private Command inner;
    private Trajectory toFollow;

    public FollowTrajectory(String pathName) {
        try {
            String filePath = Filesystem.getDeployDirectory() + "/pathplanner/trajectories/" + pathName + ".traj";

            toFollow = TrajectoryParser.parseTrajectory(filePath);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void initialize() {
        inner = AutoBuilder.buildTrajectoryFollowingCommand(toFollow);
        for (Subsystem subsystem : inner.getRequirements()) {
            addRequirements(subsystem);
        }
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
