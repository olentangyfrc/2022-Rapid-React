package frc.robot.subsystems.Climber.commands;

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
            new ExtendArmsToPosition(elevator, 2),
            new PushArmsForwardToPosition(climber, 0.9),
            new ExtendArmsToPosition(elevator, 9),
            new PushArmsForwardToPosition(climber, 0.84),
            new ExtendArmsToPosition(elevator, 7.1),
            new LetGoOfBar(climber),
            new ExtendArmsToPosition(elevator, 5),
            new PushArmsForwardToPosition(climber, 0),
            new WaitCommand(10),
            new ExtendArmsToPosition(elevator, 0.5),
            new PushArmsForwardToPosition(climber, 0.2),
            new LatchOntoBar(climber)
        );
    }
}