package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class rotateToHub extends CommandBase{

    DrivetrainSubsystem driveTrain;

    public rotateToHub(DrivetrainSubsystem driveTrain) {
        this.driveTrain = driveTrain;
    }
    
}
