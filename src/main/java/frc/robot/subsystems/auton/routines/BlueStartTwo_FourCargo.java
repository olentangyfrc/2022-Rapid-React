// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.auton.routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.auton.AutonPaths;
import frc.robot.subsystems.auton.commands.FollowTrajectoryCommand;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;
import frc.robot.subsystems.drivetrain.commands.ResetLocation;
import frc.robot.subsystems.intake.BallIntake;
import frc.robot.subsystems.intake.commands.StartIntake;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.commands.ShootBallAuton;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BlueStartTwo_FourCargo extends SequentialCommandGroup {
  /** Creates a new BlueStartOne_TwoCargo. */
  public BlueStartTwo_FourCargo(SwerveDrivetrain drivetrain, BallIntake intake, ShooterSubsystem shooter, AutonPaths paths) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ResetLocation(drivetrain, paths.getStartB2ToCargo6_2().getStartPosition()),
      new StartIntake(intake),
      new FollowTrajectoryCommand(drivetrain, paths.getStartB2ToCargo6_2()),
      new ShootBallAuton(drivetrain, shooter, intake, 2),
      new FollowTrajectoryCommand(drivetrain, paths.getCargo6ToCargo5_2()),
      new ShootBallAuton(drivetrain, shooter, intake, 1),
      new FollowTrajectoryCommand(drivetrain, paths.getCargo5ToCargo13_2()),
      new WaitCommand(1), // Wait for human player to add cargo
      new ShootBallAuton(drivetrain, shooter, intake, 2)
    );
  }
}
