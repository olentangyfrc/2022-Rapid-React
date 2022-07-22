package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.intake.BallIntake;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.SubsystemFactory;

/**
 * Speed up the shooter to a given speed and then begin feeding balls
 */
public class ShootAtSpeed extends SequentialCommandGroup {
  private ShooterSubsystem shooter;
  
  /**
   * Create a new ShootAtSpeed command
   * 
   * @param shooter The shooter subsystem to shoot
   * @param intake The intake to help transport balls to the shooter
   * @param flySpeed The speed at which to shoot in rotations per second.
   */
  public ShootAtSpeed(ShooterSubsystem shooter, BallIntake intake, double flySpeed) {
    this.shooter = shooter;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new speedUpShooter(shooter, flySpeed),
      new feedBall(shooter),
      new WaitUntilCommand(()->false) // Never ends by itself so that we can continue shooting while a button is held down.
    );
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    shooter.stop();
    shooter.stopTrigger();
    SubsystemFactory.getInstance().getLeds().setIsShooting(false);

  }
}
