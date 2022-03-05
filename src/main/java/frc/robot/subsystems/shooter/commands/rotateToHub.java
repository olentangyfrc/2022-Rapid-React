package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;

public class rotateToHub extends CommandBase{

    SwerveDrivetrain driveTrain;

    public rotateToHub(SwerveDrivetrain driveTrain) {
        this.driveTrain = driveTrain;
    }

    @Override
    public void initialize() {

    }
}
