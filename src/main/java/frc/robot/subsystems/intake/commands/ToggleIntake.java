package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intake.BallIntake;

/**
 * This command will turn on the intake if it is off, or off if it is on.
 */
public class ToggleIntake extends InstantCommand {
  private BallIntake intake;

  public ToggleIntake(BallIntake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(intake.isIntakeRunning()) {
      intake.stopIntakeMotor();
      intake.stopNoodleMotor();
    } else {
      intake.startIntakeMotor();
      intake.startNoodleMotor();
    }
  }
}
