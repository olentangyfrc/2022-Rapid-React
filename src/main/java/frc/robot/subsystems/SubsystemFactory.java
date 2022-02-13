// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

<<<<<<< HEAD
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.subsystems.Climber.Climber;
=======
import java.net.NetworkInterface;
import java.util.Collections;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.Map;
import java.util.logging.Logger;
>>>>>>> e89104b4b2c8f44eca4332e968d907dec585ad94

import edu.wpi.first.wpilibj.XboxController.Button;
// Project imports:
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.drivetrain.commands.ZeroAngle;
import frc.robot.subsystems.telemetry.Telemetry;
import frc.robot.subsystems.IO.ButtonActionType;
import frc.robot.subsystems.IO.StickButton;

/**
 * This class instantiates and initializes all of the subsystems and stores references to them.
 */
public class SubsystemFactory {

  // SubsystemFactory is a singleton, so keep a static instance.
  private static SubsystemFactory instance;

<<<<<<< HEAD
  private PowerDistribution pdp;
  private Climber climber;
=======
  private Logger logger = Logger.getLogger("Subsystem Factory");

  /**
   * Map of known bot addresses and respective types
   */
  private Map<String, BotType> allMACs = Map.of(
    "00:80:2F:30:DB:F8", BotType.COVID,
    "00:80:2F:30:DB:F9", BotType.COVID,
    "00:80:2F:25:B4:CA", BotType.CALIFORNIA,
    "00:80:2F:28:64:39", BotType.RIO99,
    "00:80:2F:28:64:38", BotType.RIO99
  );

  private BotType botType;
  private Telemetry telemetry;
>>>>>>> e89104b4b2c8f44eca4332e968d907dec585ad94

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
    botType = getBotType();
    portManager = new PortManager();
    telemetry = new Telemetry(botType);
    telemetry.init();
    io = new IO();
    io.init(botType);
    switch(botType) {
      case COVID:
        initCOVID();
        break;
      case CALIFORNIA:
        initCALIFORNIA();
        break;
      case RIO99:
        initRIO99();
        break;
      default:
        logger.info("Unrecognized bot");
    }
  }

  /**
   * Initializes COVID subsystems
   * @throws Exception
   */
  public void initCOVID() throws Exception {
    HashMap<String, Integer> portAssignments = new HashMap<String, Integer>();
    portAssignments.put("FL.SwerveMotor", 35);
    portAssignments.put("FL.DriveMotor", 34);
    portAssignments.put("FL.Encoder", 0);

    portAssignments.put("FR.SwerveMotor", 32);
    portAssignments.put("FR.DriveMotor", 33);
    portAssignments.put("FR.Encoder", 1);

    portAssignments.put("BL.SwerveMotor", 36);
    portAssignments.put("BL.DriveMotor", 37);
    portAssignments.put("BL.Encoder", 2);

    portAssignments.put("BR.SwerveMotor", 31);
    portAssignments.put("BR.DriveMotor", 30);
    portAssignments.put("BR.Encoder", 3);

    HashMap<String, Double> wheelOffsets = new HashMap<String, Double>();
    wheelOffsets.put("FL", 121.46);
    wheelOffsets.put("FR", 36.38);
    wheelOffsets.put("BL", 218.4);
    wheelOffsets.put("BR", 105.08);

    // Create and initialize all subsystems:
    driveTrain = new DrivetrainSubsystem();
    driveTrain.init(portAssignments, wheelOffsets);
  }

  /**
   * Initializes Califorinia Bot subsystems
   * @throws Exception
   */
  public void initCALIFORNIA() throws Exception{
    HashMap<String, Integer> portAssignments = new HashMap<String, Integer>();
    portAssignments.put("FL.SwerveMotor", 17);
    portAssignments.put("FL.DriveMotor", 6);
    portAssignments.put("FL.Encoder", 0);
    

    portAssignments.put("FR.SwerveMotor", 14);
    portAssignments.put("FR.DriveMotor", 9);
    portAssignments.put("FR.Encoder", 1);

    portAssignments.put("BL.SwerveMotor", 15);
    portAssignments.put("BL.DriveMotor", 10);
    portAssignments.put("BL.Encoder", 2);

    portAssignments.put("BR.SwerveMotor", 59);
    portAssignments.put("BR.DriveMotor", 60);
    portAssignments.put("BR.Encoder", 3);

    HashMap<String, Double> wheelOffsets = new HashMap<String, Double>();
    wheelOffsets.put("FL", 149.58);
    wheelOffsets.put("FR", 47.109);
    wheelOffsets.put("BL", 87.45);
    wheelOffsets.put("BR", 96.76);

    
    // Create and initialize all subsystems:
    driveTrain = new DrivetrainSubsystem();
    driveTrain.init(portAssignments, wheelOffsets);
    
    io.bind(new ZeroAngle(telemetry.getPigeon(), driveTrain), Button.kY, StickButton.RIGHT_2, ButtonActionType.WHEN_PRESSED);
  }

  /**
   * Initializes the RIO99 subsystems
   */
  public void initRIO99() {
  }
  
  // Getter methods for all of the subsystems:

  /**
   * @return The active portManager
   */
  public PortManager getPortManager() {
    return portManager;
  }

  /**
   * @return The active IO
   */
  public IO getIO() {
    return io;
  }

  /**
   * @return The active telemetry
   */
  public Telemetry getTelemetry() {
    return telemetry;
  }

  public DrivetrainSubsystem getDrivetrain() {
    return driveTrain;
  }

  /**
   * @return The active bot type
   * @throws Exception
   */
  private BotType getBotType() throws Exception {
    Enumeration<NetworkInterface> networks;
    networks = NetworkInterface.getNetworkInterfaces();
    BotType bot = BotType.UNRECOGNIZED;
    for (NetworkInterface net : Collections.list(networks)) {
        String mac = formatMACAddress(net.getHardwareAddress());
        logger.info("Network #"+net.getIndex()+" "+net.getName()+" "+mac);
        if (allMACs.containsKey(mac)) {
            bot = allMACs.get(mac);
            logger.info("   this MAC is for "+bot);
        }
    }

    return bot;
  }

  /**
   * Formats the byte array representing the mac address as more human-readable form
   * @param hardwareAddress byte array
   * @return string of hex bytes separated by colons
   */
  private String formatMACAddress(byte[] hardwareAddress) {
    if (hardwareAddress == null || hardwareAddress.length == 0) {
      return "";
    }
    StringBuilder mac = new StringBuilder(); // StringBuilder is a premature optimization here, but done as best practice
    for (int k=0;k<hardwareAddress.length;k++) {
      int i = hardwareAddress[k] & 0xFF;  // unsigned integer from byte
      String hex = Integer.toString(i,16);
      if (hex.length() == 1) {  // we want to make all bytes two hex digits 
        hex = "0"+hex;
      }
      mac.append(hex.toUpperCase());
      mac.append(":");
    }
    mac.setLength(mac.length()-1);  // trim off the trailing colon
    return mac.toString();
  }

  /**
   * Known bots to prevent user error
   */
  public enum BotType {
    COVID,
    CALIFORNIA,
    RIO99,
    UNRECOGNIZED
  }
}
