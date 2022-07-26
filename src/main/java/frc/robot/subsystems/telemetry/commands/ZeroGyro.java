// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.telemetry.commands;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Reset the gyro
 */
public class ZeroGyro extends InstantCommand {
  private Gyro gyro;

  /**
   * Construct a new ZeroGyro command
   * 
   * @param gyro the gyro to reset
   */
  public ZeroGyro(Gyro gyro) {
    this.gyro = gyro;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    gyro.reset();
  }
}
