package astrolabe;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class DriveToDistance extends ProfiledPIDCommand {
    private final DoubleSupplier getDistance;
    private final double target;
    private final Consumer<ChassisSpeeds> output;

    public DriveToDistance(
        DoubleSupplier getPose, 
        Consumer<ChassisSpeeds> output, 
        double target,
        Subsystem... requirements
    ) {
        super(
            new ProfiledPIDController(5, 0, 0, new Constraints(3, 3)),
            () -> getPose.getAsDouble(),
            target,
            (input, state) -> {
                output.accept(new ChassisSpeeds(state.velocity + input, 0, 0));
            },
            requirements
        );
        this.output = output;
        this.getDistance = getPose;
        this.target = target;
    }

    public boolean isFinished() {
        return Math.abs(getDistance.getAsDouble() - target) < Units.degreesToRadians(0.5);
    }

    public void end(boolean i) {
        output.accept(new ChassisSpeeds());
    }
}