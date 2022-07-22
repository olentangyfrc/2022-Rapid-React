package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intake.BallIntake;

/**
 * This command causes the intake's pneumatic pistons to raise it within the frame perimeter.
 */
public class BringIntakeUp extends InstantCommand {
  private BallIntake intake;

  public BringIntakeUp(BallIntake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.bringIntakeUp();
  }
}
