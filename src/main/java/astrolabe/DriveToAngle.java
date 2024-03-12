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

public class DriveToAngle extends ProfiledPIDCommand {
    private final DoubleSupplier getAngle;
    private final double target;
    private final Consumer<ChassisSpeeds> output;

    public DriveToAngle(
        Supplier<Rotation2d> getPose, 
        Consumer<ChassisSpeeds> output, 
        Rotation2d target,
        Subsystem... requirements
    ) {
        super(
            new ProfiledPIDController(5, 0, 0, new Constraints(3, 3)),
            () -> getPose.get().getRadians(),
            target.getRadians(),
            (input, state) -> {
                output.accept(new ChassisSpeeds(0, 0, state.velocity + input));
            },
            requirements
        );
        this.output = output;
        this.getAngle = () -> getPose.get().getRadians();
        this.target = target.getRadians();
    }

    public boolean isFinished() {
        return Math.abs(getAngle.getAsDouble() - target) < Units.degreesToRadians(0.5);
    }

    public void end(boolean i) {
        output.accept(new ChassisSpeeds());
    }
}