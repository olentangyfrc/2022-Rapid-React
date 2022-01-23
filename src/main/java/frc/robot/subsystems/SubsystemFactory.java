// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// WPI imports
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

// Project imports:
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

/**
 * This class instantiates and initializes all of the subsystems and stores references to them.
 */
public class SubsystemFactory {
  public static final int PDP_PORT = 1;

  // SubsystemFactory is a singleton, so keep a static instance.
  private static SubsystemFactory instance;

  private PowerDistribution pdp;

  // Variables for all subsystems:
  private PortManager portManager;
  private IO io;
  private DrivetrainSubsystem driveTrain;

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
  public void init() throws Exception {
    pdp = new PowerDistribution(PDP_PORT, ModuleType.kCTRE);

    // Create and initialize all subsystems:

    portManager = new PortManager();
    io = new IO();
    io.init();

    driveTrain = new DrivetrainSubsystem();
    driveTrain.init();

  }
  
  // Getter methods for all of the subsystems:

  public PortManager getPortManager() {
    return portManager;
  }

  public IO getIO() {
    return io;
  }
}
