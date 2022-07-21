package frc.robot.subsystems.Elevator.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator.Elevator;

/**
 * This command will extend the climbing arms slowly while it is active.
 * <p>
 * This does not end on its own
 */
public class ManualElevatorUp extends CommandBase {
  private Elevator elevator;

  /**
   * Create a new ManualElevatorUp command.
   * <p>
   * This command does not end on its own.
   * 
   * @param elevator The elevator subsystem to move
   */
  public ManualElevatorUp(Elevator elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set the target position to the maximum extended position so that the arms will start extending
    elevator.setTargetRotations(elevator.getMaxHeight());
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
