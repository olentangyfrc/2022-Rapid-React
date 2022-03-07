package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;

public class rotateToHub extends CommandBase {

    SwerveDrivetrain driveTrain;
    double angle;

    public rotateToHub(SwerveDrivetrain driveTrain) {
        this.driveTrain = driveTrain;
        SwerveDrivePoseEstimator poseEstimator = driveTrain.getposeEstimator();
        double x = poseEstimator.getEstimatedPosition().getX();
        double y = poseEstimator.getEstimatedPosition().getY();
        angle = Math.atan(Math.sqrt(Math.pow((457-x),2.0)-Math.pow((134.5-y),2.0)));
        }

    @Override
    public void initialize() {
        driveTrain.setTargetAngle(new Rotation2d(Math.toRadians(angle)));
    }
}
