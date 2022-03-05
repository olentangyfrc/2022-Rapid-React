package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.shooter.shooterSubsystem;
import frc.robot.subsystems.shooter.commands.feedBall;

public class shootBall extends SequentialCommandGroup {
    
    public shootBall(DrivetrainSubsystem driveTrain,shooterSubsystem shooterSubsystem, double flyWheelRPS) {

        addCommands(
            new prepareToShoot(driveTrain, shooterSubsystem, flyWheelRPS),
            new feedBall(shooterSubsystem)
        );
    }    

}
