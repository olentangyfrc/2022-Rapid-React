package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.shooter.shooterSubsystem;

public class prepareToShoot extends ParallelCommandGroup {
    
    public prepareToShoot(DrivetrainSubsystem driveTrain,shooterSubsystem shooterSubsystem, double flyWheelRPS) {

        addCommands(
            new rotateToHub(driveTrain),
            new speedUpShooter(shooterSubsystem, flyWheelRPS)
        );

    }

}
