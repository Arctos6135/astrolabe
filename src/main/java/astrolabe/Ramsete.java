package astrolabe;

import java.util.function.Consumer;
import java.util.function.DoubleFunction;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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

    // a function from [0, trajectory.getTotalTimeSeconds()] -> [0, 1]
    // a value of 0 means that the ramsete controller doesn't do any feedback based on position, 
    // just considering the yaw of the robot and the trajectory setpoint
    // a value of 1 means that the ramsete controller behaves as normal, consider the position and yaw
    // of the robot, as well as the trajectory setpoint
    private final DoubleFunction<Double> positionCorrection;

    public Ramsete(
        Trajectory trajectory, 
        RamseteController controller,
        Supplier<Pose2d> getPose,
        Consumer<ChassisSpeeds> output,
        DoubleFunction<Double> positionCorrection,
        Subsystem... requirements
    ) {
        this.trajectory = trajectory;
        this.controller = controller;
        this.getPose = getPose;
        this.output = output;
        this.positionCorrection = positionCorrection;

        addRequirements(requirements);
    }

    public Ramsete(
        Trajectory trajectory, 
        RamseteController controller,
        Supplier<Pose2d> getPose,
        Consumer<ChassisSpeeds> output,
        Subsystem... requirements
    ) {
        this(trajectory, controller, getPose, output, time -> 1.0, requirements);
    }

    @Override
    public void initialize() {
        System.out.println("Initializing ramsete");
        timer.restart();

        AstrolabeLogger.stateLogger.accept(1);
        AstrolabeLogger.trajectoryLogger.accept(trajectory);
    }

    @Override
    public void execute() {
        System.out.printf("Executing ramsete at timer %s\n", timer.get());
        var state = trajectory.sample(timer.get());

        Pose2d currentPose = getPose.get();
        Rotation2d currentAngle = currentPose.getRotation();

        double correctionFactor = positionCorrection.apply(timer.get());

        Translation2d adjustedTranslation = currentPose.getTranslation().times(correctionFactor).plus(state.poseMeters.getTranslation().times(1-correctionFactor));

        Pose2d adjustedPose = new Pose2d(
            adjustedTranslation,
            currentAngle
        );

        var speeds = controller.calculate(adjustedPose, state);
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
