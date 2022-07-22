package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;
import frc.robot.subsystems.drivetrain.commands.rotateToHub;
import frc.robot.subsystems.intake.BallIntake;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * Prepare to shoot at the hub by constantly rotating to the hub and speeding up the flywheel until we are both facing the hub 
 * and our flywheel is at the correct speed.
 */
public class prepareToShoot extends ParallelDeadlineGroup {
    
    public prepareToShoot(SwerveDrivetrain driveTrain, ShooterSubsystem shooterSubsystem, BallIntake intake) {
        super(
            new WaitToShoot(driveTrain, shooterSubsystem), // deadline command
            new rotateToHub(driveTrain),
            new SpeedUpForHub(shooterSubsystem)
        );
    }

}
