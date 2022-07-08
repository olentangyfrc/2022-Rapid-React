// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber.Climber;

/**
 * Manually move the arms forwards by extending the linear actuators.
 * <p>
 * This works by setting the arms' target position to the maximum forwards position until this command ends.
 * <p>
 * This command doesn't end on its own.
 */
public class ManualArmsForwards extends CommandBase {
  private Climber climber;
  /** Creates a new ManualArmsForwards. */
  public ManualArmsForwards(Climber c) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climber = c;
    addRequirements(c);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.setTargetArmPosition(climber.MAX_ARM_POSITION);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Make sure the arms actually move.
    climber.applyArmVoltage();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Set the target position to the current position so the arms don't move anymore
    climber.setTargetArmPosition(climber.getAverageArmPosition());
    climber.stopLeftLinearActuator();
    climber.stopRightLinearActuator();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
