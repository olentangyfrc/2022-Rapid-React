package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;

public class rotateToHub extends CommandBase {

    SwerveDrivetrain driveTrain;
    // Angle in degrees
    Rotation2d angle;

    public rotateToHub(SwerveDrivetrain driveTrain) {
        this.driveTrain = driveTrain;
        // Hub position in meters
        addRequirements(driveTrain);
    }
    
    @Override
    public void initialize() {
    }
    
    public void execute() {
        Translation2d hubLocation = new Translation2d(8.255, 4.103);
    
        // Bot position in meters
        Translation2d botLocation = driveTrain.getSwerveDriveOdometry().getPoseMeters().getTranslation();
    
        // Relative hub position
        Translation2d relativeHubLocation = hubLocation.minus(botLocation);

    
        angle = new Rotation2d(Math.atan2(relativeHubLocation.getY(), relativeHubLocation.getX()));
        angle = Rotation2d.fromDegrees(angle.getDegrees());
        driveTrain.setTargetAngle(angle);
        driveTrain.drive(new ChassisSpeeds(), true);
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.removeTargetAngle();
        driveTrain.stop();
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
