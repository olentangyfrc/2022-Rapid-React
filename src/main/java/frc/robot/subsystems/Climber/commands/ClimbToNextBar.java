package frc.robot.subsystems.Climber.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.commands.ExtendArmsToPosition;

import java.util.logging.Logger;

public class ClimbToNextBar extends SequentialCommandGroup{
    private Climber climber;
    private Elevator elevator;
    
    private static Logger logger = Logger.getLogger(ClimbToNextBar.class.getName());

    public ClimbToNextBar(Climber cb, Elevator el){
        climber = cb;
        elevator = el;
        addRequirements(cb, el);
        logger.info("Climb To Next Bar");

        addCommands(
            new ExtendArmsToPosition(el, 2),
            new PushArmsForwardToPosition(cb, 0),
            new ExtendArmsToPosition(el, 0),
            new PushArmsForwardToPosition(cb, 0.4),
            new ExtendArmsToPosition(el, 9),
            new PushArmsForwardToPosition(cb, 0.55),
            new ExtendArmsToPosition(el, 7.2),
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