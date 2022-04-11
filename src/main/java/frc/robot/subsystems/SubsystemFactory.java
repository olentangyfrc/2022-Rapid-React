
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.net.NetworkInterface;
import java.util.Collections;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.logging.Logger;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IO.ButtonActionType;
import frc.robot.subsystems.IO.StickButton;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Climber.ToggleLatch;
import frc.robot.subsystems.Climber.commands.NudgeArmsBackwards;
import frc.robot.subsystems.Climber.commands.NudgeArmsForwards;
import frc.robot.subsystems.Climber.commands.auton.ClimbToFinalBar;
import frc.robot.subsystems.Climber.commands.auton.ClimbToFirstBar;
import frc.robot.subsystems.Climber.commands.auton.ClimbToNextBar;
import frc.robot.subsystems.Climber.commands.auton.ReachForFirstBar;
import frc.robot.subsystems.Climber.commands.auton.ReachForLastBar;
import frc.robot.subsystems.Climber.commands.auton.ReachForNextBar;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.commands.NudgeArmsDown;
import frc.robot.subsystems.Elevator.commands.NudgeArmsUp;
import frc.robot.subsystems.drivetrain.SingleFalconDrivetrain;
import frc.robot.subsystems.drivetrain.SparkMaxDrivetrain;
// Project imports:
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;
import frc.robot.subsystems.drivetrain.commands.DisableBrakeMode;
import frc.robot.subsystems.drivetrain.commands.EnableBrakeMode;
import frc.robot.subsystems.drivetrain.commands.LockToAngle;
import frc.robot.subsystems.drivetrain.commands.RemoveLockedAngle;
import frc.robot.subsystems.intake.BallIntake;
import frc.robot.subsystems.intake.commands.BringIntakeUp;
import frc.robot.subsystems.intake.commands.PutIntakeDown;
import frc.robot.subsystems.intake.commands.StartIntake;
import frc.robot.subsystems.intake.commands.StopIntake;
import frc.robot.subsystems.music.MusicSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.commands.ShootAtSpeed;
import frc.robot.subsystems.shooter.commands.ShootNoVision1;
import frc.robot.subsystems.shooter.commands.ShootNoVision2;
import frc.robot.subsystems.shooter.commands.shootBallTeleop;
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
  private ShooterSubsystem shooter;
  private networkTables vision;
  private BallIntake ballIntake;
  private MusicSubsystem musicSubsystem;

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
    pdp = new PowerDistribution(1, ModuleType.kRev);
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
        initRAPID_REACT();
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

    portAssignments.put("BR.SwerveMotor", 3);
    portAssignments.put("BR.DriveMotor", 30);
    portAssignments.put("BR.Encoder", 2);

    HashMap<String, Double> wheelOffsets = new HashMap<String, Double>();
    wheelOffsets.put("FL", 54.58);
    wheelOffsets.put("FR", 277.12);
    wheelOffsets.put("BL", 6.06);
    wheelOffsets.put("BR", 160.48);

    
    // Create and initialize all subsystems:
    driveTrain = new SingleFalconDrivetrain();
    driveTrain.init(portAssignments, wheelOffsets);

    vision = new networkTables();
    
    io.bind(new ZeroGyro(telemetry.getGyro()), Button.kY, StickButton.RIGHT_2, ButtonActionType.WHEN_PRESSED);

    climber = new Climber();
    climber.init();

    elevator = new Elevator();
    elevator.init();

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
    wheelOffsets.put("FL", 337.5);
    wheelOffsets.put("FR", 214.54);
    wheelOffsets.put("BL", 40.16);
    wheelOffsets.put("BR", 283.18);

    // Create and initialize all subsystems:
    driveTrain = new SparkMaxDrivetrain();
    driveTrain.init(portAssignments, wheelOffsets);
    shooter = new ShooterSubsystem();
    shooter.init(botType);
    vision = new networkTables();

    io.bind(new ZeroGyro(telemetry.getGyro()), Button.kY, StickButton.RIGHT_2, ButtonActionType.WHEN_PRESSED);

  }

  /**
   * Initializes Califorinia Bot subsystems
   * @throws Exception
   */
  public void initRAPID_REACT() throws Exception{
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
    CameraServer.startAutomaticCapture(0);
    driveTrain = new SingleFalconDrivetrain();
    driveTrain.init(portAssignments, wheelOffsets);

    shooter = new ShooterSubsystem();
    shooter.init(botType);
    vision = new networkTables();

    ballIntake = new BallIntake();
    
    climber = new Climber();
    climber.init();

    elevator = new Elevator();
    elevator.init();

    List<TalonFX> musicMotors = List.of(
      new TalonFX(41),
      new TalonFX(40),
      new TalonFX(42),
      new TalonFX(43)
    );
    Map<String, String> songs = Map.of(
      "Never Gonna Give You Up", "Never Gonna Give You Up.chrp",
      "Chop Suey", "Chop Suey.chrp",
      "Wii Sports", "Wii Sports.chrp",
      "Animal Crossing", "Animal Crossing Theme.chrp"
    );
    musicSubsystem = new MusicSubsystem(musicMotors, songs);
    musicSubsystem.loadSong("Never Gonna Give You Up");

    io.bind(new shootBallTeleop(driveTrain, shooter, SubsystemFactory.getInstance().getBallIntake()), XboxController.Button.kX, StickButton.LEFT_1, ButtonActionType.WHEN_HELD);
    io.bind(new ZeroGyro(telemetry.getGyro()), Button.kY, StickButton.LEFT_1, ButtonActionType.WHEN_PRESSED);
    // Might need to be changed...
    io.bind(new StartIntake(ballIntake), Button.kRightBumper, StickButton.RIGHT_6, ButtonActionType.WHEN_PRESSED);
    io.bind(new StopIntake(ballIntake), Button.kRightBumper, StickButton.RIGHT_7, ButtonActionType.WHEN_RELEASED);

    io.bindButtonBox(new ShootAtSpeed(shooter, ballIntake, 42.6), StickButton.LEFT_6, ButtonActionType.WHEN_HELD);

    // Climber commands
    io.bindButtonBox(new ReachForFirstBar(climber, elevator), StickButton.RIGHT_3, ButtonActionType.WHEN_PRESSED);
    io.bindButtonBox(new ClimbToFirstBar(climber, elevator), StickButton.RIGHT_2, ButtonActionType.WHEN_PRESSED);

    io.bindButtonBox(new ReachForNextBar(elevator, climber), StickButton.RIGHT_1, ButtonActionType.WHEN_PRESSED);
    io.bindButtonBox(new ClimbToNextBar(climber, elevator), StickButton.RIGHT_5, ButtonActionType.WHEN_PRESSED);

    io.bindButtonBox(new ReachForLastBar(elevator, climber), StickButton.RIGHT_8, ButtonActionType.WHEN_PRESSED);
    io.bindButtonBox(new ClimbToFinalBar(climber, elevator), StickButton.RIGHT_7, ButtonActionType.WHEN_PRESSED);

    io.bindButtonBox(new NudgeArmsDown(elevator), StickButton.LEFT_9, ButtonActionType.WHEN_PRESSED);
    io.bindButtonBox(new NudgeArmsUp(elevator), StickButton.LEFT_8, ButtonActionType.WHEN_PRESSED);

    io.bindButtonBox(new NudgeArmsBackwards(climber), StickButton.LEFT_11, ButtonActionType.WHEN_PRESSED);
    io.bindButtonBox(new NudgeArmsForwards(climber), StickButton.LEFT_10, ButtonActionType.WHEN_PRESSED);

    // Lock to hanger
    io.bindButtonBox(new LockToAngle(driveTrain, Rotation2d.fromDegrees(180)), StickButton.RIGHT_4, ButtonActionType.WHEN_PRESSED);

    io.bindButtonBox(new ZeroGyro(telemetry.getGyro()), StickButton.LEFT_5, ButtonActionType.WHEN_PRESSED);
    io.bindButtonBox(new EnableBrakeMode(driveTrain), StickButton.LEFT_1, ButtonActionType.WHEN_PRESSED);
    io.bindButtonBox(new DisableBrakeMode(driveTrain), StickButton.LEFT_4, ButtonActionType.WHEN_PRESSED);

    io.bindButtonBox(new PutIntakeDown(ballIntake), StickButton.LEFT_2, ButtonActionType.WHEN_PRESSED);
    io.bindButtonBox(new BringIntakeUp(ballIntake), StickButton.LEFT_3, ButtonActionType.WHEN_PRESSED);

    // Shooter
    // Eject ball
    io.bindButtonBox(new ShootAtSpeed(shooter, ballIntake, 27), StickButton.LEFT_7, ButtonActionType.WHEN_HELD);
    io.bindButtonBox(new ToggleLatch(climber), StickButton.RIGHT_11, ButtonActionType.WHEN_PRESSED);

    // Shoot from tarmac edge
    io.bind(new ShootNoVision1(driveTrain, shooter, ballIntake), Button.kB, StickButton.LEFT_10, ButtonActionType.WHEN_HELD);
    io.bind(new ShootNoVision2(driveTrain, shooter, ballIntake), Button.kA, StickButton.LEFT_11, ButtonActionType.WHEN_HELD);

    io.bindButtonBox(new RemoveLockedAngle(driveTrain), StickButton.RIGHT_11, ButtonActionType.WHEN_PRESSED);
    io.bindButtonBox(new InstantCommand() {
      @Override
      public void initialize() {
        CommandScheduler.getInstance().cancelAll();
      }
    }, StickButton.RIGHT_10, ButtonActionType.WHEN_PRESSED);
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

  public PowerDistribution getPdp() {
    return pdp;
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

  /**
   * @return The active drivetrain
   */
  public SwerveDrivetrain getDrivetrain() {
    return driveTrain;
  }

  public networkTables getVision() {
    return vision;
  }

  public BallIntake getBallIntake() {
    return ballIntake;
  }

  /**
   * @return the active shooter
   */
  public ShooterSubsystem getShooter() {
    return shooter;
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
