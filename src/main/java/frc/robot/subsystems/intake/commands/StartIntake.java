package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intake.BallIntake;


/**
 * This command will turn on the motors necessary to intake balls through the intake
 */
public class StartIntake extends InstantCommand {
  private BallIntake intake;

  public StartIntake(BallIntake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.startIntakeMotor();
    intake.startNoodleMotor();
  }
}
