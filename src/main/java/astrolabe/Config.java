package astrolabe;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;

public class Config {
    public final double maxAcceleration;
    public final double maxVelocity;
    public final double maxCentripetal;
    public final DifferentialDriveKinematics kinematics;

    public Config(
        double maxAcceleration,
        double maxVelocity,
        double maxCentripetal,
        DifferentialDriveKinematics kinematics
    ) {
        this.maxAcceleration = maxAcceleration;
        this.maxVelocity = maxVelocity;
        this.maxCentripetal = maxCentripetal;
        this.kinematics = kinematics;
    }

    public TrajectoryConfig trajectoryConfig(boolean reversed) {
        var config = new TrajectoryConfig(maxVelocity, maxAcceleration);
        config.addConstraint(new CentripetalAccelerationConstraint(maxCentripetal));
        config.setReversed(reversed);
        config.setKinematics(kinematics);

        return config;
    }
}
