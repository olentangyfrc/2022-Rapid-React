package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intake.BallIntake;

/**
 * This command will start the noodle motor
 */
public class StopNoodleMotor extends InstantCommand {
  private BallIntake intake;

  public StopNoodleMotor(BallIntake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.stopNoodleMotor();
  }
}
