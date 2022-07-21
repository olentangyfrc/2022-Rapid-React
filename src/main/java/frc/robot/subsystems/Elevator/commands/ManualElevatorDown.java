package frc.robot.subsystems.Elevator.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator.Elevator;

/**
 * This command will retract the climbing arms slowly while it is active.
 * <p>
 * This does not end on its own
 */
public class ManualElevatorDown extends CommandBase {
  private Elevator elevator;

  /**
   * Create a new ManualElevatorDown command
   * 
   * @param elevator The elevator subsystem to move
   */
  public ManualElevatorDown(Elevator elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set the target position to 0 so that the arms start retracting.
    elevator.setTargetRotations(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Set the target position to whatever the current position is so the elevator will no longer move.
    elevator.setTargetRotations(elevator.getPosition());
  }

  /**
   * This command does not end on its own
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}
