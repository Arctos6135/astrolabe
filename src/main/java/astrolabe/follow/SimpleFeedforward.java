package astrolabe.follow;

import java.util.function.Consumer;
import java.util.function.DoubleFunction;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class SimpleFeedforward extends Command {
    private final Trajectory trajectory;
    private final Timer timer = new Timer();

    private final Supplier<Pose2d> getPose;
    private final Consumer<ChassisSpeeds> output;

    // a function from [0, trajectory.getTotalTimeSeconds()] -> [0, 1]
    // a value of 0 means that the ramsete controller doesn't do any feedback based on position, 
    // just considering the yaw of the robot and the trajectory setpoint
    // a value of 1 means that the ramsete controller behaves as normal, consider the position and yaw
    // of the robot, as well as the trajectory setpoint

    public SimpleFeedforward(
        Trajectory trajectory, 
        Supplier<Pose2d> getPose,
        Consumer<ChassisSpeeds> output,
        Subsystem... requirements
    ) {
        this.trajectory = trajectory;
        this.getPose = getPose;
        this.output = output;

        addRequirements(requirements);

    }

    @Override
    public void initialize() {
        System.out.println("Initializing simple feedforward");
        timer.restart();

        AstrolabeLogger.stateLogger.accept(1);
        AstrolabeLogger.trajectoryLogger.accept(trajectory);
    }

    @Override
    public void execute() {
        var state = trajectory.sample(timer.get());

        var speeds = new ChassisSpeeds(state.velocityMetersPerSecond, 0, state.curvatureRadPerMeter * state.velocityMetersPerSecond);
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
        AstrolabeLogger.stateLogger.accept(0);
    }
}
