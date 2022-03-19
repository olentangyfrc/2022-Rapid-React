package frc.robot.subsystems.Climber.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.commands.ExtendArmsToPosition;

import java.util.logging.Logger;

public class ClimbToFirstBar extends SequentialCommandGroup{
    private Climber climber;
    private Elevator elevator;

    private static Logger logger = Logger.getLogger(ClimbToFirstBar.class.getName());
    
    public ClimbToFirstBar(Climber cb, Elevator el){
        climber = cb;
        elevator = el;
        addRequirements(cb, el);
        logger.info("Climb to First Bar");

        addCommands(
            new ExtendArmsToPosition(elevator, 3),
            new PushArmsForwardToPosition(climber, 0),
            new ExtendArmsToPosition(elevator, 0.5),
            new PushArmsForwardToPosition(climber, 0.2),
            new LatchOntoBar(climber)
        );
    }
}