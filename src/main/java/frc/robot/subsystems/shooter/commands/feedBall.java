package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class feedBall extends CommandBase {

    ShooterSubsystem shooterSubsystem;

    public feedBall(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }
    
    @Override
    public void initialize() {
        shooterSubsystem.shoot();
    }
    
}
