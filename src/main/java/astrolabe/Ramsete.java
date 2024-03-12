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

    private static Supplier<Pose2d> getPose;
    private static Consumer<ChassisSpeeds> output;

    public Ramsete(Trajectory trajectory, RamseteController controller, Subsystem... requirements) {
        this.trajectory = trajectory;
        this.controller = controller;
        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        timer.reset();
    }

    @Override
    public void execute() {
        var state = trajectory.sample(timer.get());
        var speeds = controller.calculate(getPose.get(), state);
        output.accept(speeds);
    }

    @Override
    public boolean isFinished() {
        return timer.get() > trajectory.getTotalTimeSeconds();
    }

    @Override
    public void end(boolean i) {
        output.accept(new ChassisSpeeds());
    }
}
