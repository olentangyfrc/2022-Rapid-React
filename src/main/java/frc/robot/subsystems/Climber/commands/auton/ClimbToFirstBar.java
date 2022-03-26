package frc.robot.subsystems.Climber.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Climber.commands.LatchOntoBar;
import frc.robot.subsystems.Climber.commands.PushArmsForwardToPosition;
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
            new ExtendArmsToPosition(elevator, 0.1),
            new PushArmsForwardToPosition(climber, 0.089),
            new LatchOntoBar(climber)
        );
    }
}