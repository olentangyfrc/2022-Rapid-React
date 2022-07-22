package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubsystemFactory;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * Speed up our flywheel to the correct speed for shooting into the hub.
 * <p>
 * This command does not end on its own and must be interrupted.
 */
public class SpeedUpForHub extends CommandBase {

    ShooterSubsystem shooterSubsystem;

    public SpeedUpForHub(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void execute() {
        double distance = SubsystemFactory.getInstance().getVision().getDistanceFromHub();

        // We did a linear regression to determine the relationship between distance to hub and optimal flywheel speed
        shooterSubsystem.setSpeed(distance * 4.8771 + 29.143);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
