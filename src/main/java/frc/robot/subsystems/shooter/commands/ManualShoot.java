package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;
import frc.robot.subsystems.drivetrain.commands.RotateToAngle;
import frc.robot.subsystems.intake.BallIntake;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * Rotate the drivetrain to a given angle and shoot a ball at a given speed.
 */
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
    // Unlock the target angle
    drivetrain.removeTargetAngle();
  }
}
