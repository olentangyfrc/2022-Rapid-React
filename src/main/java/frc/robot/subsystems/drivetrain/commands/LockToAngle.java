package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;

/**
 * This command causes the drivetrain to lock to an angle and begin trying to rotate to that angle.
 * <p>
 * This removes user control of the robot's angle until the locked angle is removed..
 */
public class LockToAngle extends InstantCommand {
  private SwerveDrivetrain drivetrain;
  private Rotation2d angle;

  public LockToAngle(SwerveDrivetrain drivetrain, Rotation2d angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.angle = angle;
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.setTargetAngle(angle);
  }
}
