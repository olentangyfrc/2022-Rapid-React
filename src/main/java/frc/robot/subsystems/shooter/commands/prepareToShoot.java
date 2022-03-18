package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class prepareToShoot extends ParallelCommandGroup {
    
    public prepareToShoot(SwerveDrivetrain driveTrain,ShooterSubsystem shooterSubsystem, double flyWheelRPS) {

        addCommands(
            new rotateToHub(driveTrain),
            new speedUpShooter(shooterSubsystem, flyWheelRPS)
        );

    }

}
