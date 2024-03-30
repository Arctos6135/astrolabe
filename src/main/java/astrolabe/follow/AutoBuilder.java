package astrolabe.follow;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class AutoBuilder {
    private static boolean isConfigured = false;

    private static Function<AstrolabePath, Command> commandBuilder;
    private static Supplier<Pose2d> getPose;
    private static Supplier<ChassisSpeeds> getSpeeds;
    private static Consumer<Pose2d> resetPose;
    private static Consumer<ChassisSpeeds> output;
    public static Subsystem[] requirements;
    private static BooleanSupplier shouldFlipPath;

    public static GlobalConfig config;

    public static void configureRamsete(
        RamseteController controller,
        Supplier<Pose2d> getPose,
        Supplier<ChassisSpeeds> getSpeeds,
        Consumer<Pose2d> resetPose,
        Consumer<ChassisSpeeds> output,
        GlobalConfig config,
        BooleanSupplier shouldFlipPath,
        Subsystem... requirements
    ) {
        if (isConfigured) {
            DriverStation.reportError("Attempted to configure astrolabe AutoBuilder when it was already configured.", new StackTraceElement[]{});
            return;
        }
        isConfigured = true;

        commandBuilder = path -> {
            var trajectory = path.generateTrajectory(config);
            return new Ramsete(trajectory, controller, getPose, output, requirements);
        };

        AutoBuilder.getPose = getPose;
        AutoBuilder.getSpeeds = getSpeeds;
        AutoBuilder.resetPose = resetPose;
        AutoBuilder.output = output;
        AutoBuilder.config = config;
        AutoBuilder.requirements = requirements;
        AutoBuilder.shouldFlipPath = shouldFlipPath;
    }

    public static void configureRamseteRefine(
        RamseteController controller,
        Supplier<Pose2d> getPose,
        Supplier<ChassisSpeeds> getSpeeds,
        Consumer<Pose2d> resetPose,
        Consumer<ChassisSpeeds> output,
        GlobalConfig config,
        BooleanSupplier shouldFlipPath,
        Subsystem... requirements
    ) {
        if (isConfigured) {
            DriverStation.reportError("Attempted to configure astrolabe AutoBuilder when it was already configured.", new StackTraceElement[]{});
            return;
        }
        isConfigured = true;

        // commandBuilder = path -> {
        //     var trajectory = path.generateTrajectory(config);
        //     return new Refine(
        //         refinedPath -> {
        //             var refinedTraj = refinedPath.generateTrajectory(config);
        //             return new Ramsete(refinedTraj, controller, getPose, output, requirements);
        //         }, path, getPose, output, requirements
        //     );
        // };
        commandBuilder = path -> {
            var trajectory = path.generateTrajectory(config);
            return new Ramsete(
                trajectory,
                controller,
                getPose,
                output,
                time -> (1 - time / trajectory.getTotalTimeSeconds()),
                requirements
            );
        };

        AutoBuilder.getPose = getPose;
        AutoBuilder.getSpeeds = getSpeeds;
        AutoBuilder.resetPose = resetPose;
        AutoBuilder.output = output;
        AutoBuilder.requirements = requirements;
        AutoBuilder.shouldFlipPath = shouldFlipPath;
    }

    public static Command buildPathFollowingCommand(AstrolabePath path) {
        return commandBuilder.apply(path);
    }

    public static Command buildTrajectoryFollowingCommand(Trajectory trajectory) {
        if (shouldFlipPath.getAsBoolean()) {
            trajectory = flipTrajectory(trajectory);
        }
        return new Ramsete(trajectory, new RamseteController(), getPose, output, requirements);
    }

    private static Trajectory flipTrajectory(Trajectory trajectory) {
        ArrayList<State> states = new ArrayList<>();

        for (State state : trajectory.getStates()) {
            Translation2d flippedTranslation = new Translation2d(16.54 - state.poseMeters.getX(), state.poseMeters.getY());
            Rotation2d flippedRotation = Rotation2d.fromDegrees(180).minus(state.poseMeters.getRotation());
            Pose2d flippedPose = new Pose2d(flippedTranslation, flippedRotation);
            State flipped = new State(state.timeSeconds, state.velocityMetersPerSecond, state.accelerationMetersPerSecondSq, flippedPose, -state.curvatureRadPerMeter);

            states.add(flipped);
        }
        
        return new Trajectory(states);
    }
}