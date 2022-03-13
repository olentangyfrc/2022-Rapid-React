// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.SubsystemFactory;
import frc.robot.subsystems.auton.AutonChooser;
import frc.robot.subsystems.auton.commands.FollowTrajectoryCommand;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private AutonChooser chooser;
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  
  @Override
  public void robotInit() {

    
    try {
        SubsystemFactory.getInstance().init();
      } catch (Exception exception) {
          exception.printStackTrace();
    }
    chooser = new AutonChooser();

  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    SubsystemFactory.getInstance().getDrivetrain().resetLocation(new Pose2d(7.492, 1.779, Rotation2d.fromDegrees(100)));

    TrajectoryConfig config = new TrajectoryConfig(SwerveDrivetrain.MAX_LINEAR_SPEED, SwerveDrivetrain.MAX_LINEAR_ACCELERATION);


    try {
      Trajectory trajectory = chooser.getTrajectory();
      (new FollowTrajectoryCommand(SubsystemFactory.getInstance().getDrivetrain(), trajectory, Rotation2d.fromDegrees(0))).schedule();
    } catch(Exception ex) {
      ex.printStackTrace();
    }
  }

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

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {
    CommandScheduler.getInstance().run();
  }
}
