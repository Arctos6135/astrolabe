package astrolabe;

import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class AutoBuilder {
    private static boolean isConfigured = false;

    private static Function<AstrolabePath, Command> commandBuilder;
    private static Supplier<Pose2d> getPose;
    private static Supplier<ChassisSpeeds> getSpeeds;
    private static Consumer<Pose2d> resetPose;
    private static Consumer<ChassisSpeeds> output;

    public static void configureRamsete(
        RamseteController controller,
        Supplier<Pose2d> getPose,
        Supplier<ChassisSpeeds> getSpeeds,
        Consumer<Pose2d> resetPose,
        Consumer<ChassisSpeeds> output,
        GlobalConfig config,
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
    }

    public static void configureRamseteRefine(
        RamseteController controller,
        Supplier<Pose2d> getPose,
        Supplier<ChassisSpeeds> getSpeeds,
        Consumer<Pose2d> resetPose,
        Consumer<ChassisSpeeds> output,
        GlobalConfig config,
        Subsystem... requirements
    ) {
        if (isConfigured) {
            DriverStation.reportError("Attempted to configure astrolabe AutoBuilder when it was already configured.", new StackTraceElement[]{});
            return;
        }
        isConfigured = true;

        commandBuilder = path -> {
            var trajectory = path.generateTrajectory(config);
            return new Refine(
                refinedPath -> {
                    var refinedTraj = refinedPath.generateTrajectory(config);
                    return new Ramsete(refinedTraj, controller, getPose, output, requirements);
                }, path, getPose, output, requirements
            );
        };

        AutoBuilder.getPose = getPose;
        AutoBuilder.getSpeeds = getSpeeds;
        AutoBuilder.resetPose = resetPose;
        AutoBuilder.output = output;
    }

    public static Command buildPathFollowingCommand(AstrolabePath path) {
        return commandBuilder.apply(path);
    }
}