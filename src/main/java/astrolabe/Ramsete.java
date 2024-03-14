package astrolabe;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Ramsete extends Command {
    private final Trajectory trajectory;
    private final RamseteController controller;
    private final Timer timer = new Timer();

    private final Supplier<Pose2d> getPose;
    private final Consumer<ChassisSpeeds> output;

    public Ramsete(
        Trajectory trajectory, 
        RamseteController controller,
        Supplier<Pose2d> getPose,
        Consumer<ChassisSpeeds> output,
        Subsystem... requirements
    ) {
        this.trajectory = trajectory;
        this.controller = controller;
        this.getPose = getPose;
        this.output = output;

        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        System.out.println("Initializing ramsete");
        timer.restart();

        AstrolabeLogger.stateLogger.accept(1);
    }

    @Override
    public void execute() {
        System.out.printf("Executing ramsete at timer %s\n", timer.get());
        var state = trajectory.sample(timer.get());
        var speeds = controller.calculate(getPose.get(), state);
        output.accept(speeds);

        AstrolabeLogger.targetPoseLogger.accept(state.poseMeters);
    }

    @Override
    public boolean isFinished() {
        return timer.get() > trajectory.getTotalTimeSeconds();
    }

    @Override
    public void end(boolean i) {
        System.out.println("Ramsete finished");
        output.accept(new ChassisSpeeds());
    }
}
