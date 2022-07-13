package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;

/**
 * This command causes the drivetrain to stop locking to an angle.
 * <p>
 * This will return user control of the robot's angle.
 */
public class RemoveLockedAngle extends InstantCommand {
  private SwerveDrivetrain drivetrain;

  public RemoveLockedAngle(SwerveDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.removeTargetAngle();
  }
}
