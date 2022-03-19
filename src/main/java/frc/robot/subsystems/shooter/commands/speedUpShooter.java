package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class speedUpShooter extends CommandBase {

    ShooterSubsystem shooterSubsystem;
    double flyWheelRPS;

    public speedUpShooter(ShooterSubsystem shooterSubsystem, double flyWheelRPS) {
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
        return shooterSubsystem.isReady();
    }
}