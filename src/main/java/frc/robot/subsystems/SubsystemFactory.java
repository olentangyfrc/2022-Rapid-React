
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.net.NetworkInterface;
import java.util.Collections;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.Map;
import java.util.logging.Logger;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IO.ButtonActionType;
import frc.robot.subsystems.IO.StickButton;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Climber.ClimberSBTab;
import frc.robot.subsystems.Climber.commands.ClimbToFirstBar;
import frc.robot.subsystems.Climber.commands.ClimbToNextBar;
import frc.robot.subsystems.Climber.commands.LatchOntoBar;
import frc.robot.subsystems.Climber.commands.LetGoOfBar;
import frc.robot.subsystems.Climber.commands.PullArmsBack;
import frc.robot.subsystems.Climber.commands.PushArmsForward;
import frc.robot.subsystems.Climber.commands.PushArmsForwardToPosition;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.commands.ExtendArms;
import frc.robot.subsystems.Elevator.commands.ExtendArmsToPosition;
import frc.robot.subsystems.Elevator.commands.RetractArms;
import frc.robot.subsystems.drivetrain.SingleFalconDrivetrain;
import frc.robot.subsystems.drivetrain.SparkMaxDrivetrain;
// Project imports:
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;
import frc.robot.subsystems.telemetry.Telemetry;
import frc.robot.subsystems.telemetry.commands.ZeroGyro;

/**
 * This class instantiates and initializes all of the subsystems and stores references to them.
 */
public class SubsystemFactory {

  // SubsystemFactory is a singleton, so keep a static instance.
  private static SubsystemFactory instance;

  private PowerDistribution pdp;
  private Climber climber;
  private Elevator elevator;
  private Logger logger = Logger.getLogger("Subsystem Factory");

  /**
   * Map of known bot addresses and respective types
   */
  private Map<String, BotType> allMACs = Map.of(
    "00:80:2F:30:DB:F8", BotType.COVID,
    "00:80:2F:30:DB:F9", BotType.COVID,
    "00:80:2F:25:B4:CA", BotType.RAPID_REACT,
    "00:80:2F:28:64:39", BotType.RIO99,
    "00:80:2F:28:64:38", BotType.RIO99,
    "00:80:2F:17:F8:3F", BotType.RIO1, //radio
    "00:80:2F:17:F8:40", BotType.RIO1, //usb
    "00:80:2F:17:D7:4B", BotType.RIO2,
    "00:80:2F:27:04:C7", BotType.RIO3,
    "00:80:2F:27:04:C6", BotType.RIO3
  );

  private BotType botType;
  private Telemetry telemetry;

  // Variables for all subsystems:
  private PortManager portManager;
  private IO io;
  private SwerveDrivetrain driveTrain;

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
    io.init();
    switch(botType) {
      case COVID:
        initCOVID();
        break;
      case RAPID_REACT:
        initRapidReact();
        break;
      case RIO99:
        initRIO99();
        break;
      case RIO1:
        initRIO1();
        break;
      default:
        logger.info("Unrecognized bot");
    }
  }

  private NetworkTableEntry targetArmPosition = Shuffleboard.getTab("Climber").add("Target Arm position", 0.5).withWidget(BuiltInWidgets.kTextView).getEntry();
  private NetworkTableEntry targetElevatorPosition = Shuffleboard.getTab("Climber").add("Target Elevator position", 0.5).withWidget(BuiltInWidgets.kTextView).getEntry();

  private void initRIO1() throws Exception {
    HashMap<String, Integer> portAssignments = new HashMap<String, Integer>();
    portAssignments.put("FL.SwerveMotor", 9);
    portAssignments.put("FL.DriveMotor", 32);
    portAssignments.put("FL.Encoder", 0);
    

    portAssignments.put("FR.SwerveMotor", 11);
    portAssignments.put("FR.DriveMotor", 31);
    portAssignments.put("FR.Encoder", 1);

    portAssignments.put("BL.SwerveMotor", 6);
    portAssignments.put("BL.DriveMotor", 33);
    portAssignments.put("BL.Encoder", 3);

    portAssignments.put("BR.SwerveMotor", 14);
    portAssignments.put("BR.DriveMotor", 30);
    portAssignments.put("BR.Encoder", 2);

    HashMap<String, Double> wheelOffsets = new HashMap<String, Double>();
    wheelOffsets.put("FL", 60.54);
    wheelOffsets.put("FR", 277.64);
    wheelOffsets.put("BL", 80.6);
    wheelOffsets.put("BR", 160.92);

    
    // Create and initialize all subsystems:
    driveTrain = new SingleFalconDrivetrain();
    driveTrain.init(portAssignments, wheelOffsets);
    
    io.bind(new ZeroGyro(telemetry.getGyro()), Button.kY, StickButton.RIGHT_2, ButtonActionType.WHEN_PRESSED);

    climber = new Climber();
    elevator = new Elevator();

    logger.info("Initializing Climber Subsystem");
    climber.init();
    elevator.init();
    ClimberSBTab climberTab = new ClimberSBTab(climber, elevator);
    io.bind(new PushArmsForward(climber), Button.kLeftBumper, StickButton.LEFT_6, ButtonActionType.WHEN_HELD);
    io.bind(new PullArmsBack(climber), Button.kRightBumper, StickButton.LEFT_7, ButtonActionType.WHEN_HELD);
    io.bind(new ExtendArms(elevator), Button.kB, StickButton.LEFT_8, ButtonActionType.WHEN_HELD);
    io.bind(new RetractArms(elevator), Button.kRightStick, StickButton.LEFT_9, ButtonActionType.WHEN_HELD);
    io.bind(new LatchOntoBar(climber), Button.kX, StickButton.LEFT_10, ButtonActionType.WHEN_PRESSED);
    io.bind(new LetGoOfBar(climber), Button.kA, StickButton.LEFT_11, ButtonActionType.WHEN_PRESSED);

    Shuffleboard.getTab("Climber").add("Climb to first bar", new ClimbToFirstBar(climber, elevator));
    Shuffleboard.getTab("Climber").add("Climb to next bar", new ClimbToNextBar(climber, elevator));
    // io.bind(new ClimbToFirstBar(climber, elevator), Button.k, StickButton.RIGHT_11, ButtonActionType.WHEN_PRESSED);
    
    io.bind(new ExtendArmsToPosition(elevator, 10.3), Button.kBack, StickButton.RIGHT_8, ButtonActionType.WHEN_PRESSED);
    io.bind(new ExtendArmsToPosition(elevator, 0), Button.kStart, StickButton.RIGHT_9, ButtonActionType.WHEN_HELD);
    Shuffleboard.getTab("Climber").add("Move Arms", new InstantCommand() {
      @Override
      public void initialize() {
        (new PushArmsForwardToPosition(climber, targetArmPosition.getDouble(0.5))).schedule();
      }
    });

    Shuffleboard.getTab("Climber").add("Move Elevator", new InstantCommand() {
      @Override
      public void initialize() {
        (new ExtendArmsToPosition(elevator, targetElevatorPosition.getDouble(0.0))).schedule();
      }
    });

    //io.bind(new ExtendArmsToPosition(elevator, 3), Button.kX, StickButton.RIGHT_10, ButtonActionType.WHEN_PRESSED);
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
    wheelOffsets.put("FL", 336.7);
    wheelOffsets.put("FR", 214.31);
    wheelOffsets.put("BL", 40.13);
    wheelOffsets.put("BR", 283.46);

    // Create and initialize all subsystems:
    driveTrain = new SparkMaxDrivetrain();
    driveTrain.init(portAssignments, wheelOffsets);

    io.bind(new ZeroGyro(telemetry.getGyro()), Button.kY, StickButton.RIGHT_2, ButtonActionType.WHEN_PRESSED);

  }

  /**
   * Initializes Rapid React Competition Bot subsystems
   * @throws Exception
   */
  public void initRapidReact() throws Exception{
    HashMap<String, Integer> portAssignments = new HashMap<String, Integer>();
    portAssignments.put("FL.SwerveMotor", 59);
    portAssignments.put("FL.DriveMotor", 41);
    portAssignments.put("FL.Encoder", 1);
    

    portAssignments.put("FR.SwerveMotor", 8);
    portAssignments.put("FR.DriveMotor", 40);
    portAssignments.put("FR.Encoder", 3);

    portAssignments.put("BL.SwerveMotor", 17);
    portAssignments.put("BL.DriveMotor", 42);
    portAssignments.put("BL.Encoder", 2);

    portAssignments.put("BR.SwerveMotor", 15);
    portAssignments.put("BR.DriveMotor", 43);
    portAssignments.put("BR.Encoder", 0);

    HashMap<String, Double> wheelOffsets = new HashMap<String, Double>();
    wheelOffsets.put("FL", 51.1);
    wheelOffsets.put("FR", 322.77);
    wheelOffsets.put("BL", 293.53);
    wheelOffsets.put("BR", 249.6);

    
    // Create and initialize all subsystems:
    driveTrain = new SingleFalconDrivetrain();
    driveTrain.init(portAssignments, wheelOffsets);
    
    io.bind(new ZeroGyro(telemetry.getGyro()), Button.kY, StickButton.RIGHT_2, ButtonActionType.WHEN_PRESSED);

    climber = new Climber();
    elevator = new Elevator();

    logger.info("Initializing Climber Subsystem");
    climber.init();
    elevator.init();
    ClimberSBTab climberTab = new ClimberSBTab(climber, elevator);
    io.bind(new PushArmsForward(climber), Button.kLeftBumper, StickButton.LEFT_6, ButtonActionType.WHEN_HELD);
    io.bind(new PullArmsBack(climber), Button.kRightBumper, StickButton.LEFT_7, ButtonActionType.WHEN_HELD);
    io.bind(new ExtendArms(elevator), Button.kB, StickButton.LEFT_8, ButtonActionType.WHEN_PRESSED);
    io.bind(new RetractArms(elevator), Button.kRightStick, StickButton.LEFT_9, ButtonActionType.WHEN_PRESSED);
    io.bind(new LatchOntoBar(climber), Button.kX, StickButton.LEFT_10, ButtonActionType.WHEN_PRESSED);
    io.bind(new LetGoOfBar(climber), Button.kA, StickButton.LEFT_11, ButtonActionType.WHEN_PRESSED);

    Shuffleboard.getTab("Climber").add("Climb to first bar", new ClimbToFirstBar(climber, elevator));
    Shuffleboard.getTab("Climber").add("Climb to next bar", new ClimbToNextBar(climber, elevator));
    // io.bind(new ClimbToFirstBar(climber, elevator), Button.k, StickButton.RIGHT_11, ButtonActionType.WHEN_PRESSED);
    
    io.bind(new ExtendArmsToPosition(elevator, 10.3), Button.kBack, StickButton.RIGHT_8, ButtonActionType.WHEN_PRESSED);
    io.bind(new ExtendArmsToPosition(elevator, 0), Button.kStart, StickButton.RIGHT_9, ButtonActionType.WHEN_HELD);
    Shuffleboard.getTab("Climber").add("Move Arms", new InstantCommand() {
      @Override
      public void initialize() {
        (new PushArmsForwardToPosition(climber, targetArmPosition.getDouble(0.5))).schedule();
      }
    });

    Shuffleboard.getTab("Climber").add("Move Elevator", new InstantCommand() {
      @Override
      public void initialize() {
        (new ExtendArmsToPosition(elevator, targetElevatorPosition.getDouble(0.0))).schedule();
      }
    });

    //io.bind(new ExtendArmsToPosition(elevator, 3), Button.kX, StickButton.RIGHT_10, ButtonActionType.WHEN_PRESSED);
  }

  /**
   * Initializes the RIO99 subsystems
   */
  public void initRIO99() {
    
  }

  /**
   * Initializes the RIO2 subsystems
   */
  public void initRIO2() {
    
  }

  /**
   * Initializes the RIO3 subsystems
   */
  public void initRIO3() {
    
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

  public SwerveDrivetrain getDrivetrain() {
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
    RAPID_REACT,
    RIO99,
    RIO1,
    RIO2, 
    RIO3,
    UNRECOGNIZED
  }
}
