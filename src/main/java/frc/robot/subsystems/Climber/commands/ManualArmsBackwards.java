// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber.Climber;

/**
 * Manually move the arms backwards by retracting the linear actuators.
 * <p>
 * This works by setting the arms' target position to the minimum forwards position until this command ends.
 * <p>
 * This command doesn't end on its own.
 */
public class ManualArmsBackwards extends CommandBase {
  private Climber c;

  /** Creates a new ManualArmsBackwards. */
  public ManualArmsBackwards(Climber c) {
    this.c = c;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(c);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    c.setTargetArmPosition(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {  
    // Make sure that the arms actually move 
    c.applyArmVoltage();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Set the target position to the current position so the arms don't move anymore
    c.setTargetArmPosition(c.getAverageArmPosition());
    c.stopLeftLinearActuator();
    c.stopRightLinearActuator();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
