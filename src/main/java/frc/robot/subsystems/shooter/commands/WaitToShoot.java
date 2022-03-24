// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.commands;

import java.time.Instant;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class WaitToShoot extends CommandBase {
  private SwerveDrivetrain drivetrain;
  private ShooterSubsystem shooter;

  private Instant startTime;

  /** Creates a new WaitToShoot. */
  public WaitToShoot(SwerveDrivetrain drivetrain, ShooterSubsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Instant.now();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putBoolean("At target angle", drivetrain.atTargetAngle());
    SmartDashboard.putBoolean("Shooter at speed", shooter.isReady());
    return drivetrain.atTargetAngle() && shooter.isReady();
  }
}
