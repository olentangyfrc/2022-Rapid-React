package frc.robot.subsystems.shooter.commands;

import java.time.Instant;
import java.time.temporal.ChronoUnit;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class takeInBall extends CommandBase {

    private ShooterSubsystem shooterSubsystem;
    private Instant startTime;

    public takeInBall(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void initialize() {
        shooterSubsystem.setTriggerVoltage(0.55);
        startTime = Instant.now();
    }

    @Override
    public void end(boolean interupted) {
        shooterSubsystem.setTriggerVoltage(0);
    }

    @Override
    public boolean isFinished() {
        return !shooterSubsystem.isTriggerMoving() && ChronoUnit.MILLIS.between(startTime, Instant.now()) > 250;
    }
}
