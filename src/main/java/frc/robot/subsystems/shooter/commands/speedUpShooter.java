package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * Speed up the shooter to a given speed
 */
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

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooter.isReady();
  }
}
