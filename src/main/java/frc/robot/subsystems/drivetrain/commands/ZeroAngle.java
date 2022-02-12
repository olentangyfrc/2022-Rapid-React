// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.telemetry.commands.ZeroGyro;

/**
 * Should be called to zero gyro angle when there is a drivetrain.
 */
public class ZeroAngle extends SequentialCommandGroup {
  /** Creates a new ZeroAngle. */
  public ZeroAngle(Gyro gyro, DrivetrainSubsystem drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ZeroGyro(gyro),
      new ResetAngleController(drivetrain)
    );
  }
}
