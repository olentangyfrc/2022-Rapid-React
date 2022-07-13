package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;

/**
 * This will reset the estimated location of the drivetrain to a given Pose2d.
 */
public class ResetLocation extends InstantCommand {
  private SwerveDrivetrain drivetrain;
  private Pose2d newPose;

  public ResetLocation(SwerveDrivetrain drivetrain, Pose2d newPose) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.newPose = newPose;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.resetLocation(newPose);
  }
}
