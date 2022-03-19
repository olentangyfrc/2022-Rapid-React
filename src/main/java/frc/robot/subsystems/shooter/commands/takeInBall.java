package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.BallIntake;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class takeInBall extends CommandBase {

    private ShooterSubsystem shooterSubsystem;
    private BallIntake intake;

    public takeInBall(ShooterSubsystem shooterSubsystem, BallIntake intake) {
        this.shooterSubsystem = shooterSubsystem;
        this.intake = intake;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.takeInBall();
        intake.startNoodleMotor();
    }

    @Override
    public void end(boolean interupted) {
        shooterSubsystem.stopTrigger();
        shooterSubsystem.setBallLoaded(true);
        intake.stopNoodleMotor();
    }

    @Override
    public boolean isFinished() {
        return shooterSubsystem.getTriggerSpeed() == 0 && shooterSubsystem.getPreviousTriggerSpeed() != 0;
    }
}
