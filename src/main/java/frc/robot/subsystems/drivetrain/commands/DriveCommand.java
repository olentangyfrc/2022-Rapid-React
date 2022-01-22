// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubsystemFactory;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.IO;

public class DriveCommand extends CommandBase {

  private DrivetrainSubsystem drivetrain;

  /** Creates a new DriveCommand. */
  public DriveCommand(DrivetrainSubsystem drivetrain) {
    this.drivetrain = drivetrain;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    IO io = SubsystemFactory.getInstance().getIO();
    ChassisSpeeds speeds = new ChassisSpeeds(
      io.getForward(),
      io.getStrafe(),
      io.getRotation()
    );

    drivetrain.drive(speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Always return false because this will be the drivetrain's default command.
  @Override
  public boolean isFinished() {
    return false;
  }
}
