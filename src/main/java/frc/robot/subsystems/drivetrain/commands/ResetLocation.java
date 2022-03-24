// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetLocation extends InstantCommand {
  private SwerveDrivetrain drivetrain;
  private Pose2d newPose;

  public ResetLocation(SwerveDrivetrain drivetrain, Pose2d newPose) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.newPose = newPose;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.resetLocation(newPose);
  }
}
