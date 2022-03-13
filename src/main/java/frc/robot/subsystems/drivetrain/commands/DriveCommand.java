// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IO;
import frc.robot.subsystems.SubsystemFactory;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;

/**
 * This command will grab user input from IO and call the drivetrain's drive method.
 * This should be called periodically by the drivetrain.
 */
public class DriveCommand extends CommandBase {
  private SwerveDrivetrain drivetrain;
  private SlewRateLimiter xLimiter = new SlewRateLimiter(2);
  private SlewRateLimiter yLimiter = new SlewRateLimiter(2);
  private SlewRateLimiter thetaLimiter = new SlewRateLimiter(2);

  public DriveCommand(SwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    IO io = SubsystemFactory.getInstance().getIO();
    ChassisSpeeds speeds = new ChassisSpeeds(
      xLimiter.calculate(io.getForward()) * SwerveDrivetrain.MAX_LINEAR_SPEED,
      yLimiter.calculate(io.getStrafe()) * SwerveDrivetrain.MAX_LINEAR_SPEED,
      thetaLimiter.calculate(io.getRotation()) * SwerveDrivetrain.MAX_ROTATION_SPEED
    );

    drivetrain.drive(speeds, drivetrain.getFieldOriented());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
