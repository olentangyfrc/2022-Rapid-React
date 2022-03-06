package frc.robot.subsystems.Climber.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber.Climber;

import java.util.logging.Logger;

public class ClimbToFirstBar extends SequentialCommandGroup{
    private Climber climber;

    private static Logger logger = Logger.getLogger(ClimbToFirstBar.class.getName());
    
    public ClimbToFirstBar(Climber cb){
        climber = cb;
        addRequirements(cb);
        logger.info("Climb to First Bar");

        addCommands(
            new PushArmsForwardToPosition(climber, 0.29, 0.39),
            //new ExtendArmsToPosition(climber, -10),
            new PullArmsBackToPosition(climber, 0.19, 0.3),
            new RetractArmsToPosition(climber, -0.5),
            new PushArmsForwardToPosition(climber, 0.28, 0.37),
            new LatchOntoBar(climber)
        );
    }
}