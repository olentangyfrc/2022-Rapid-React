// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.IO;
import frc.robot.subsystems.SubsystemFactory;
import frc.robot.subsystems.IO.ButtonActionType;
import frc.robot.subsystems.IO.StickButton;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.commands.shootBall;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  ShooterSubsystem shooter;
  SwerveDrivetrain drivetrain;
  IO io;

  @Override
  public void robotInit() {

    
    try {
      SubsystemFactory.getInstance().init();
      shooter = SubsystemFactory.getInstance().getShooter();
      drivetrain = SubsystemFactory.getInstance().getDrivetrain();
      io = SubsystemFactory.getInstance().getIO();
      io.bind(new shootBall(drivetrain, shooter, 20), XboxController.Button.kX, StickButton.LEFT_1, ButtonActionType.WHILE_HELD);
    } catch (Exception exception) {
      exception.printStackTrace();
    }
    

  }

  @Override
  public void robotPeriodic() {
    actualSpeedEntry.setNumber(shooter.getFlySpeed());
    currentPosition.setNumber(shooter.getFlyPosition());
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}


  private NetworkTableEntry targetSpeedEntry = Shuffleboard.getTab("Shooter").add("Target Speed", 0).withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("min", -60, "max", 60)).getEntry();
  private NetworkTableEntry actualSpeedEntry = Shuffleboard.getTab("Shooter").add("Actual Speed", 0).withWidget(BuiltInWidgets.kGraph).withProperties(Map.of("min", 0, "max", 0)).getEntry();
  private NetworkTableEntry currentPosition = Shuffleboard.getTab("Shooter").add("Current Position", 0).withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("min",0,"max",0)).getEntry();

  private NetworkTableEntry shootBall = Shuffleboard.getTab("Shooter").add("Shoot Ball", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
  private NetworkTableEntry triggerOff = Shuffleboard.getTab("Shooter").add("Stop Trigger", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {

    shooter.periodic();
    
    if (shootBall.getBoolean(false)) {
      shooter.shoot();
      shootBall.setBoolean(false);
    }

    if (triggerOff.getBoolean(false)) {
      shooter.stopTrigger();
      triggerOff.setBoolean(false);
    }

    shooter.setSpeed(targetSpeedEntry.getDouble(0));

    CommandScheduler.getInstance().run();
  }
}
