// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SubsystemFactory;
import frc.robot.subsystems.IO.ButtonActionType;
import frc.robot.subsystems.IO.StickButton;
import frc.robot.subsystems.auton.AutonPaths;
import frc.robot.subsystems.auton.routines.RoutineChooser;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;
import frc.robot.subsystems.drivetrain.commands.ResetLocation;
import frc.robot.subsystems.intake.BallIntake;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.commands.ShootAtSpeed;
import frc.robot.subsystems.shooter.commands.ShootBallAuton;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private RoutineChooser chooser;
  private CommandBase autonCommand;

  private NetworkTableEntry shooterSpeedEntry = Shuffleboard.getTab("Shooter").add("Test Shooter Speed", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
  
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
    ShooterSubsystem shooter = SubsystemFactory.getInstance().getShooter();
    SwerveDrivetrain drivetrain = SubsystemFactory.getInstance().getDrivetrain();
    BallIntake intake = SubsystemFactory.getInstance().getBallIntake();
    AutonPaths paths = new AutonPaths(new TrajectoryConfig(SwerveDrivetrain.MAX_LINEAR_SPEED - 1, SwerveDrivetrain.MAX_LINEAR_ACCELERATION));

    chooser = new RoutineChooser(drivetrain, shooter, intake, paths);

    SubsystemFactory.getInstance().getIO().bindButtonBox(new InstantCommand() {
      @Override
      public void initialize() {
        (new ShootAtSpeed(shooter, intake, shooterSpeedEntry.getDouble(0.0))).schedule();
      }
    }, StickButton.RIGHT_3, ButtonActionType.WHEN_HELD);

  }

  @Override
  public void robotPeriodic() {
  }


  @Override
  public void autonomousInit() {
    autonCommand = chooser.get();
    autonCommand.schedule();
    SubsystemFactory.getInstance().getBallIntake().putIntakeDown();

    // (new ResetLocation(SubsystemFactory.getInstance().getDrivetrain(), new Pose2d(6.716, 2.487, Rotation2d.fromDegrees(46.762)))).schedule();
    // (new ShootBallAuton(SubsystemFactory.getInstance().getDrivetrain(), SubsystemFactory.getInstance().getShooter(), SubsystemFactory.getInstance().getBallIntake(), 200)).schedule();
  }

  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    if(autonCommand != null) {
      autonCommand.cancel();
    }
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
