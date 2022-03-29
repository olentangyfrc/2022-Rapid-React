// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class speedUpShooter extends CommandBase {
  private ShooterSubsystem shooter;
  private double flySpeed;

  /**
   * Creates a new SpeedUpShooter command
   * 
   * @param shooter the shooter subsystem
   * @param flySpeed flywheel speed in rps
   */
  public speedUpShooter(ShooterSubsystem shooter, double flySpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.flySpeed = flySpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setSpeed(flySpeed);
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
    SmartDashboard.putNumber("Shooter speed", shooter.getFlySpeed());
    SmartDashboard.putNumber("Target shooter speed: ", flySpeed);

    return shooter.isReady();
  }
}
