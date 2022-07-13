package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;

/**
 * This is a command that will enable Brake Mode for the drivetrain. In this mode, wheels are locked pointing towards the center
 * of the robot making it difficult to move in any direction.
 */
public class EnableBrakeMode extends InstantCommand {
  private SwerveDrivetrain drivetrain;

  public EnableBrakeMode(SwerveDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.\
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.enableBrakeMode();
  }
}
