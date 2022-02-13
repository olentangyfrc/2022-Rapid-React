// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IO;
import frc.robot.subsystems.SubsystemFactory;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

/**
 * This command will grab user input from IO and call the drivetrain's drive method.
 * This should be called periodically by the drivetrain.
 */
public class DriveCommand extends InstantCommand {
  private DrivetrainSubsystem drivetrain;

  public DriveCommand(DrivetrainSubsystem drivetrain) {
    this.drivetrain = drivetrain;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    IO io = SubsystemFactory.getInstance().getIO();
    ChassisSpeeds speeds = new ChassisSpeeds(
      io.getForward(),
      io.getStrafe(),
      io.getRotation()
    );

    drivetrain.drive(speeds);
  }
}
