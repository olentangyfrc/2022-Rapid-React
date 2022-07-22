package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intake.BallIntake;

/**
 * This command causes the intake's pneumatic pistons to extend it so that it can pick up balls
 */
public class PutIntakeDown extends InstantCommand {
  private BallIntake intake;

  public PutIntakeDown(BallIntake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.putIntakeDown();
  }
}
