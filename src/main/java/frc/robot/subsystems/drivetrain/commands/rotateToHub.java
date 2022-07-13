package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;

/**
 * Cause the robot to lock its angle to the position of the hub with a slight offset to prevent balls from bouncing out.
 * <p>
 * This command does not end on its own.
 */
public class rotateToHub extends CommandBase {

    SwerveDrivetrain driveTrain;
    // Angle in degrees to try and rotate to. This is continuously calculated.
    Rotation2d angle;

    public rotateToHub(SwerveDrivetrain driveTrain) {
        this.driveTrain = driveTrain;
        // Hub position in meters
        addRequirements(driveTrain);
    }
    
    public void execute() {
        Translation2d hubLocation = new Translation2d(8.255, 4.103);
    
        // Bot position in meters
        Translation2d botLocation = driveTrain.getSwerveDriveOdometry().getPoseMeters().getTranslation();
    
        // Relative hub position
        Translation2d relativeHubLocation = hubLocation.minus(botLocation);

        //Apply a slight offset
        Translation2d normal_left = new Translation2d( -relativeHubLocation.getY(),  relativeHubLocation.getX());
        Translation2d offset = normal_left.div(normal_left.getNorm()).times(0.2);
        relativeHubLocation = relativeHubLocation.plus(offset);
    
        // Calculate relative angle
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
