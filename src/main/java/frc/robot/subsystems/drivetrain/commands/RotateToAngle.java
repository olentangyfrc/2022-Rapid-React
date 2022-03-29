// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;

public class RotateToAngle extends CommandBase {
  private SwerveDrivetrain drivetrain;
  private Rotation2d angle;
  /** Creates a new RotateToAngle. */
  public RotateToAngle(SwerveDrivetrain drivetrain, Rotation2d angle) {
    this.drivetrain = drivetrain;
    this.angle = angle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.setTargetAngle(angle);
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(new ChassisSpeeds(), true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.removeTargetAngle();
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drivetrain.atTargetAngle();
  }
}
