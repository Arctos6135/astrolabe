package astrolabe;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class AngleDistanceRamsete extends Command {
    private final double targetDistance;
    private final Rotation2d targetAngle;
    private final double initialSpeed;
    private final RamseteController controller;
    private final Supplier<Pose2d> getPose;
    private final Consumer<ChassisSpeeds> output;



    private final TrapezoidProfile angleProfile;
    private final TrapezoidProfile distanceProfile;

    private State angleInitial;
    private final State angleGoal;

    private final State distanceInitial;
    private final State distanceGoal;

    private final Timer timer = new Timer();


    public AngleDistanceRamsete(
        double targetDistance,
        Rotation2d targetAngle,
        double initialSpeed,
        RamseteController controller,
        Supplier<Pose2d> getPose,
        Consumer<ChassisSpeeds> output
    ) {
        this.targetDistance = targetDistance;
        this.targetAngle = targetAngle;
        this.initialSpeed = initialSpeed;
        this.controller = controller;
        this.getPose = getPose;
        this.output = output;

        angleProfile = new TrapezoidProfile(new Constraints(AutoBuilder.config.maxVelocity, AutoBuilder.config.maxAcceleration));
        distanceProfile = new TrapezoidProfile(new Constraints(AutoBuilder.config.maxVelocity, AutoBuilder.config.maxAcceleration));

        angleGoal = new State(0, 0);
        
        distanceInitial = new State(0, initialSpeed);
        distanceGoal = new State(targetDistance, 0);
    }

    @Override
    public void initialize() {
        angleInitial = new State(getPose.get().getRotation().getRadians(), 0);
        timer.restart();
    }

    @Override
    public void execute() {
        var currentPose = getPose.get();
        var targetPose = new Pose2d(currentPose.getTranslation(), targetAngle);

        var time = timer.get();
        var angleState = angleProfile.calculate(time, angleInitial, angleGoal);
        var distanceState = distanceProfile.calculate(time, distanceInitial, distanceGoal);

        output.accept(controller.calculate(currentPose, targetPose, angleState.velocity, distanceState.velocity));
    }

    @Override
    public void end(boolean i) {
        output.accept(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        return distanceProfile.isFinished(timer.get());
    }
}
