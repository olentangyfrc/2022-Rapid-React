// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.auton.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.auton.AutonTrajectoryFollower;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;

public class FollowTrajectoryCommand extends CommandBase {
  
  private SwerveDrivetrain drivetrain;
  private Trajectory trajectory;
  private AutonTrajectoryFollower follower;
  private Rotation2d targetAngle;

  // The time that the bot starts following the trajectory in seconds
  private double startTime;

  /**
   * Creates a new FollowTrajectoryCommand
   * 
   * @param drivetrain the drivetrain that will follow the trajectory
   * @param trajectory the trajectory to follow
   * @param targetAngle the desired angle at the end of the trajectory
   */
  public FollowTrajectoryCommand(SwerveDrivetrain drivetrain, Trajectory trajectory, Rotation2d targetAngle) {

    this.drivetrain = drivetrain;
    this.trajectory = trajectory;
    this.targetAngle = targetAngle;

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
      drivetrain::getLocation,
      targetAngle
    );

    startTime = (double) System.currentTimeMillis() / 1000;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(follower.calculate(getCurrentTIme()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return follower.hasFinished(getCurrentTIme());
  }

  /**
   * Get the time since the start in seconds
   * 
   * @return the time since the start in seconds.
   */
  private double getCurrentTIme() {
    return ((double) System.currentTimeMillis() / 1000) - startTime;
  }
}
