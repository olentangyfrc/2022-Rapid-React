package frc.robot.subsystems.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;

public class AutonPaths {

    private AutonTrajectory start1ToCargo2;

    private AutonTrajectory start2ToCargo3;
    private AutonTrajectory cargo3ToCargo4;

    private AutonTrajectory start3ToCargo4;
    private AutonTrajectory cargo4ToCargo3;

    public AutonPaths(TrajectoryConfig config) {

    }

    private void loadTrajectories(TrajectoryConfig config) {
        start1ToCargo2 = new AutonTrajectory(
            new Pose2d(6.65, 5.287, Rotation2d.fromDegrees(36.3488)),
            new Pose2d(5.363, 5.947, Rotation2d.fromDegrees(217.9085)),
            config
        );

        start2ToCargo3 = new AutonTrajectory(
            new Pose2d(7.037, 2.611, Rotation2d.fromDegrees(307.6476)),
            new Pose2d(5.591, 2.042, Rotation2d.fromDegrees(127.6476)),
            config
        );

        cargo3ToCargo4 = new AutonTrajectory(
            new Pose2d(5.591, 2.042, Rotation2d.fromDegrees(127.6476)),
            new Pose2d(5.591, 2.042, Rotation2d.fromDegrees(127.6476)),
            config
        );
    }
}
