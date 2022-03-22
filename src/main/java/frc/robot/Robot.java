// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.SubsystemFactory;
import frc.robot.subsystems.auton.AutonTrajectory;
import frc.robot.subsystems.auton.commands.FollowTrajectoryCommand;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;
import frc.robot.subsystems.shooter.commands.takeInBall;

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

  @Override
  public void robotInit() {

    
    try {
      SubsystemFactory.getInstance().init();
      
    } catch (Exception exception) {
      exception.printStackTrace();
    }
    

  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
    SubsystemFactory.getInstance().getBallIntake().putIntakeDown();
    (new takeInBall(SubsystemFactory.getInstance().getShooter())).schedule();

    SubsystemFactory.getInstance().getDrivetrain().resetLocation(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

    TrajectoryConfig config = new TrajectoryConfig(SwerveDrivetrain.MAX_LINEAR_SPEED, SwerveDrivetrain.MAX_LINEAR_ACCELERATION);

    //AutonTrajectory trajectory = new AutonTrajectory(new Pose2d(8.266, 2.052, Rotation2d.fromDegrees(90.509)), new Pose2d(7.893, 1.307, Rotation2d.fromDegrees(249.535)), config);
    AutonTrajectory trajectory = new AutonTrajectory(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), new Pose2d(3, 0, Rotation2d.fromDegrees(0)), config);
    System.out.println("SEGMENTS IN TRAJECTORY: " + trajectory.getSegments().size());
    try {
      (new FollowTrajectoryCommand(SubsystemFactory.getInstance().getDrivetrain(), trajectory)).schedule();
    } catch(Exception ex) {
      ex.printStackTrace();
    }
  }

  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    (new takeInBall(SubsystemFactory.getInstance().getShooter())).schedule(); // COMP!!!
  }

  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
  }
  
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}



  @Override
  public void testInit() {
  }
  
  @Override
  public void testPeriodic() {
    SubsystemFactory.getInstance().getBallIntake().bringIntakeUp();
    CommandScheduler.getInstance().run();
  }
}
