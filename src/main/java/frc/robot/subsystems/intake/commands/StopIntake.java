package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intake.BallIntake;

/**
 * This command will turn off the motors necessary to intake balls through the intake
 */
public class StopIntake extends InstantCommand {
  private BallIntake intake;

  public StopIntake(BallIntake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.stopIntakeMotor();
    intake.stopNoodleMotor();
  }
}
