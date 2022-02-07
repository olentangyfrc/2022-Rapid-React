// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

/**
 * Reset the storedAngle of the anglePid controller to 0.
 */
public class ResetAngleController extends InstantCommand {
  private DrivetrainSubsystem drivetrainSubsystem;

  public ResetAngleController(DrivetrainSubsystem drivetrainSubsystem) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrainSubsystem.resetAngleController();
  }
}
