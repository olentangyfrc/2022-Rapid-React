package frc.robot.subsystems.Climber.commands.auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Climber.commands.LetGoOfBar;
import frc.robot.subsystems.Climber.commands.PushArmsForwardToPosition;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.commands.ExtendArmsToPosition;

public class ClimbToNextBar extends SequentialCommandGroup{
    
    public ClimbToNextBar(Climber cb, Elevator el) {
        addCommands(
            new LetGoOfBar(cb),
            new ParallelCommandGroup(
                new ExtendArmsToPosition(el, 5),
                new SequentialCommandGroup(
                    new WaitCommand(0.4),
                    new PushArmsForwardToPosition(cb, 0)
                )
            )
        );
    }
}