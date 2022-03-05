package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.shooterSubsystem;

public class speedUpShooter extends CommandBase {

    shooterSubsystem shooterSubsystem;
    double flyWheelRPS;

    public speedUpShooter(shooterSubsystem shooterSubsystem, double flyWheelRPS) {
        this.shooterSubsystem = shooterSubsystem;
        this.flyWheelRPS = flyWheelRPS;
    
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.setSpeed(flyWheelRPS);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
