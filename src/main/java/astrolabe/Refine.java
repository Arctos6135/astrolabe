package astrolabe;

import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Refine extends SequentialCommandGroup {
    public Refine(
        Function<AstrolabePath, Command> toRefine, 
        AstrolabePath path,
        Supplier<Pose2d> getPose,
        Consumer<ChassisSpeeds> output,
        Subsystem... requirements
    ) {
        double refineDistance = 1;
        Pose2d refinedEnd;
        Rotation2d endAngle = path.endPose().getRotation();
        Translation2d endTranslation = path.endPose().getTranslation();
        Translation2d refinedDelta = new Translation2d(refineDistance, endAngle);

        if (path.reversed()) {
            refinedEnd = new Pose2d(endTranslation.plus(refinedDelta), endAngle);
        } else {
            refinedEnd = new Pose2d(endTranslation.minus(refinedDelta), endAngle);
        }

        var mainPath = toRefine.apply(new AstrolabePath(path.startPose(), path.waypoints(), refinedEnd, path.reversed()));

        addCommands(
            mainPath,
            new DriveToAngle(() -> getPose.get().getRotation(), output, endAngle, requirements),
            new DriveToDistance(() -> getPose.get().getTranslation().getDistance(refinedEnd.getTranslation()), output, refineDistance, requirements)
        );
    }
}
