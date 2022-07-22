package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intake.BallIntake;

/**
 * This command will start the external motor on the intake
 */
public class StopIntakeMotor extends InstantCommand {
  private BallIntake intake;

  public StopIntakeMotor(BallIntake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.stopIntakeMotor();
  }
}
