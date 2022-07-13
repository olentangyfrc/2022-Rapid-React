package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IO;
import frc.robot.subsystems.SubsystemFactory;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;

/**
 * This command will repeatedly grab user input from IO and call the drivetrain's drive method.
 */
public class DriveCommand extends CommandBase {
  private SwerveDrivetrain drivetrain;

  public DriveCommand(SwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
  
  public void execute() {
    IO io = SubsystemFactory.getInstance().getIO();
    // Get inputs from io and convert percentages to actual speeds.
    ChassisSpeeds speeds = new ChassisSpeeds(
      io.getForward() * SwerveDrivetrain.MAX_LINEAR_SPEED,
      io.getStrafe() * SwerveDrivetrain.MAX_LINEAR_SPEED,
      io.getRotation() * SwerveDrivetrain.MAX_ROTATION_SPEED
    );
    
    drivetrain.drive(speeds, !io.isLeftBumperPressed()); // Holding left bumper will cause the robot to drive in bot-oriented mode

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
