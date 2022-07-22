package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intake.BallIntake;

/**
 * This command will turn on the noodle motor if it is off, or off if it is on.
 */
public class ToggleNoodleMotor extends InstantCommand {
  private BallIntake intake;

  public ToggleNoodleMotor(BallIntake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(intake.isNoodleRunning()) {
      intake.stopNoodleMotor();
    } else {
      intake.startNoodleMotor();
    }
  }
}
