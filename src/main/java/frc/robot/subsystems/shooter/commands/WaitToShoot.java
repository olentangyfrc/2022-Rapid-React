// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.commands;

import java.time.Instant;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubsystemFactory;
import frc.robot.subsystems.networkTables;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class WaitToShoot extends CommandBase {
  private SwerveDrivetrain drivetrain;
  private ShooterSubsystem shooter;

  private double startTimeSeconds;

  /** Creates a new WaitToShoot. */
  public WaitToShoot(SwerveDrivetrain drivetrain, ShooterSubsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTimeSeconds = Timer.getFPGATimestamp();
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
    SmartDashboard.putBoolean("Vision Ready", (networkTables.getlaststabletime()>startTimeSeconds));

    return drivetrain.atTargetAngle() && shooter.isReady() 
    
    &&  (networkTables.getlaststabletime()>startTimeSeconds);
  }
}
