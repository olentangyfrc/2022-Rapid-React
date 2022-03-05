package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.shooterSubsystem;

public class feedBall extends CommandBase {

    shooterSubsystem shooterSubsystem;

    public feedBall(shooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }
    
    @Override
    public void initialize() {
        shooterSubsystem.shoot();
    }
    
}
