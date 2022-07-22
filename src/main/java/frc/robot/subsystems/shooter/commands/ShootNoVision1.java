package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;
import frc.robot.subsystems.drivetrain.commands.RotateToAngle;
import frc.robot.subsystems.intake.BallIntake;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * This command allows us to shoot from a predefined position, in this case from the left corner of the tarmac.
 * <p>
 * This is useful if we have issues with our vision
 */
public class ShootNoVision1 extends SequentialCommandGroup {
  private SwerveDrivetrain drivetrain;

  /** Creates a new ShootNoVision. */
  public ShootNoVision1(SwerveDrivetrain drivetrain, ShooterSubsystem shooter, BallIntake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.drivetrain = drivetrain;
    addCommands(
      new RotateToAngle(drivetrain, Rotation2d.fromDegrees(335.339)),
      new ShootAtSpeed(shooter, intake, 42.6)
    );

  }
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    drivetrain.removeTargetAngle();
  }
}
