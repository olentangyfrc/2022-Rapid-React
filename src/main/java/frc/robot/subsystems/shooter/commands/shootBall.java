package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class shootBall extends SequentialCommandGroup {
    
    public shootBall(SwerveDrivetrain driveTrain,ShooterSubsystem shooterSubsystem, double flyWheelRPS) {

        addCommands(
            new prepareToShoot(driveTrain, shooterSubsystem, flyWheelRPS),
            new takeInBall(shooterSubsystem),
            new feedBall(shooterSubsystem)
        );
    }    

}
