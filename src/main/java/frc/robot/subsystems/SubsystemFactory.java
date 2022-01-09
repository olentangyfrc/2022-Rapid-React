// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistributionPanel;

// Subsystem imports:

public class SubsystemFactory {

  // SubsystemFactory is a singleton, so keep a static instance.
  private static SubsystemFactory instance;

  private PowerDistributionPanel pdp;

  // Variables for all subsystems:
  private PortManager portManager;

  // Should not be used outside of this class!
  private SubsystemFactory() {}

  /**
   * Get the instance of the SubsystemFactory class.
   * If there is no instance, create one, otherwise return the already-existing one.
   * 
   * @return the instance of SubsystemFactory
   */
  public static SubsystemFactory getInstance() {
    if(instance == null) {
      instance = new SubsystemFactory();
    }
    return instance;
  }

  /**
   * Create and initialize all of the subsystems.
   */
  public void init() {
    pdp = new PowerDistributionPanel();

    // Create and initialize all subsystems:

    portManager = new PortManager();

  }
  
  // Getter methods for all of the subsystems:

  public PortManager getPortManager() {
    return portManager;
  }
}
