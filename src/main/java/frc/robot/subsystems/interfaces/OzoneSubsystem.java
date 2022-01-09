// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.logging.Logger;

/**
 * This class is meant to add additional functionality to WPI's SubsystemBase class.
 */
public abstract class OzoneSubsystem extends SubsystemBase {
  // Use this for logging information to the RioLog and DriverStation
  protected Logger logger = Logger.getLogger(this.getClass().getName());

  /** Creates a new OzoneSubsystem. */
  public OzoneSubsystem() {}

  public abstract void init();

  
}
