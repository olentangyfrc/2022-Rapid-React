// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;
import frc.robot.subsystems.drivetrain.commands.RotateToAngle;
import frc.robot.subsystems.intake.BallIntake;
import frc.robot.subsystems.shooter.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ManualShoot extends SequentialCommandGroup {
  private SwerveDrivetrain drivetrain;

  /** Creates a new ManualShoot. */
  public ManualShoot(SwerveDrivetrain drivetrain, ShooterSubsystem shooter, BallIntake intake, Rotation2d angle, double flySpeed) {
    this.drivetrain = drivetrain;
    addCommands(
      new RotateToAngle(drivetrain, angle),
      new ShootAtSpeed(shooter, intake, flySpeed)
    );

  }
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    drivetrain.removeTargetAngle();
  }
}
