package frc.robot.subsystems.Climber.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Elevator.Elevator;

import java.util.logging.Logger;

public class ClimbToNextBar extends SequentialCommandGroup{
    private Climber climber;
    
    private static Logger logger = Logger.getLogger(ClimbToNextBar.class.getName());

    public ClimbToNextBar(Climber cb, Elevator el){
        climber = cb;
        addRequirements(cb);
        logger.info("Climb To Next Bar");

        addCommands(
            //new ExtendArmsToPosition(climber, -1.71),
            new PushArmsForwardToPosition(climber, 0.98, 1.06),
            //new ExtendArmsToPosition(climber, -8.27),
            new PullArmsBackToPosition(climber, 0.86, 0.95),
            new ExtendArmsToPosition(el, -6.79),
            new LetGoOfBar(climber),
            new ExtendArmsToPosition(el, -5.57),
            new PushArmsForwardToPosition(climber, 0.19, 0.3),
            new ExtendArmsToPosition(el, -0.69),
            new PushArmsForwardToPosition(climber, 0.37, 0.46),
            new LatchOntoBar(climber)
        );
    }
}