// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;
import frc.robot.subsystems.drivetrain.commands.LockToAngle;
import frc.robot.subsystems.intake.BallIntake;
import frc.robot.subsystems.shooter.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootNoVision1 extends SequentialCommandGroup {
  private SwerveDrivetrain drivetrain;

  /** Creates a new ShootNoVision. */
  public ShootNoVision1(SwerveDrivetrain drivetrain, ShooterSubsystem shooter, BallIntake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.drivetrain = drivetrain;
    addCommands(
      new LockToAngle(drivetrain, Rotation2d.fromDegrees(335.339)),
      new ShootAtSpeed(shooter, intake, 42.6)
    );

  }
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    drivetrain.removeTargetAngle();
  }
}
