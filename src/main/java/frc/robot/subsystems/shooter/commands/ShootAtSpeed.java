// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.intake.BallIntake;
import frc.robot.subsystems.shooter.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootAtSpeed extends SequentialCommandGroup {
  private ShooterSubsystem shooter;
  private double flySpeed;


  /** Creates a new EjectBall. */
  public 
  ShootAtSpeed(ShooterSubsystem shooter, BallIntake intake, double flySpeed) {
    this.shooter = shooter;
    this.flySpeed = flySpeed;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new speedUpShooter(shooter, flySpeed),
      new feedBall(shooter),
      new WaitUntilCommand(()->false) // Never ends by itself
    );
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    shooter.stop();
    shooter.stopTrigger();
  }
}
