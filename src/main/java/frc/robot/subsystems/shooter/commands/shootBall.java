package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;
import frc.robot.subsystems.shooter.shooterSubsystem;

public class shootBall extends SequentialCommandGroup {
    
    public shootBall(SwerveDrivetrain driveTrain,shooterSubsystem shooterSubsystem, double flyWheelRPS) {

        addCommands(
            new prepareToShoot(driveTrain, shooterSubsystem, flyWheelRPS),
            new feedBall(shooterSubsystem)
        );
    }    

}
