package frc.robot.subsystems.Climber.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
            new ExtendArmsToPosition(elevator, 2),
            new PushArmsForwardToPosition(climber, 0.98, 1.06),
            new ExtendArmsToPosition(elevator, 8),
            new PullArmsBackToPosition(climber, 0.86, 0.95),
            new ExtendArmsToPosition(elevator, -6.79),
            new LetGoOfBar(climber),
            new ExtendArmsToPosition(elevator, -5.57),
            new PullArmsBackToPosition(climber, 0.19, 0.3),
            new ExtendArmsToPosition(elevator, -0.69),
            new PushArmsForwardToPosition(climber, 0.37, 0.46),
            new LatchOntoBar(climber)
        );
    }
}