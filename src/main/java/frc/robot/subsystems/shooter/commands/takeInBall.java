package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class takeInBall extends CommandBase {

    ShooterSubsystem shooterSubsystem;

    public takeInBall(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.takeInBall();
    }

    @Override
    public void end(boolean interupted) {
        shooterSubsystem.stopTrigger();
    }

    @Override
    public boolean isFinished() {
        return shooterSubsystem.hasBall();
    }
}
