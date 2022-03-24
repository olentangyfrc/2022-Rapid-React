// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Climber.commands.LatchOntoBar;
import frc.robot.subsystems.Climber.commands.PushArmsForwardToPosition;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.commands.ExtendArmsToPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ReachForLastBar extends SequentialCommandGroup {
  /** Creates a new ReachForLastBar. */
  public ReachForLastBar(Elevator e, Climber c) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ExtendArmsToPosition(e, 0.8),
      new PushArmsForwardToPosition(c, 0.08),
      new LatchOntoBar(c),
      new ExtendArmsToPosition(e, 2),
      new PushArmsForwardToPosition(c, 0),
      new ExtendArmsToPosition(e, 0),
      new PushArmsForwardToPosition(c, 0.4),
      new ExtendArmsToPosition(e, 9),
      new PushArmsForwardToPosition(c, 0.6),
      new ExtendArmsToPosition(e, 7.2)
    );
  }
}