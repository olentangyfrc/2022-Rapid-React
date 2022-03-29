// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.auton.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.auton.AutonTrajectory;
import frc.robot.subsystems.auton.AutonTrajectoryFollower;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;

public class FollowTrajectoryCommand extends CommandBase {
  
  private SwerveDrivetrain drivetrain;
  private AutonTrajectory trajectory;
  private AutonTrajectoryFollower follower;
  
  // The time that the bot starts following the trajectory in seconds
  private double startTime;

  /**
   * Creates a new FollowTrajectoryCommand
   * 
   * @param drivetrain the drivetrain that will follow the trajectory
   * @param trajectory the trajectory to follow
   * @param targetAngle the desired angle at the end of the trajectory
   */
  public FollowTrajectoryCommand(SwerveDrivetrain drivetrain, AutonTrajectory trajectory) {
    this.drivetrain = drivetrain;
    this.trajectory = trajectory;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    follower = new AutonTrajectoryFollower(
      drivetrain.xController,
      drivetrain.yController,
      drivetrain.thetaController,
      trajectory,
      drivetrain::getLocation
    );

    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds speeds = follower.calculate(getCurrentTime());
    drivetrain.drive(speeds, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return follower.hasFinished(getCurrentTime());
  }

  /**
   * Get the time since the start in seconds
   * 
   * @return the time since the start in seconds.
   */
  private double getCurrentTime() {
    return Timer.getFPGATimestamp() - startTime;
  }
}
