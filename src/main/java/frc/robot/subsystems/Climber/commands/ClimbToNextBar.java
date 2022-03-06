package frc.robot.subsystems.Climber.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber.Climber;

import java.util.logging.Logger;

public class ClimbToNextBar extends SequentialCommandGroup{
    private Climber climber;
    
    private static Logger logger = Logger.getLogger(ClimbToNextBar.class.getName());

    public ClimbToNextBar(Climber cb){
        climber = cb;
        addRequirements(cb);
        logger.info("Climb To Next Bar");

        addCommands(
            //new ExtendArmsToPosition(climber, -1.71),
            new PushArmsForwardToPosition(climber, 0.98, 1.06),
            //new ExtendArmsToPosition(climber, -8.27),
            new PullArmsBackToPosition(climber, 0.86, 0.95),
            new RetractArmsToPosition(climber, -6.79),
            new LetGoOfBar(climber),
            new RetractArmsToPosition(climber, -5.57),
            new PushArmsForwardToPosition(climber, 0.19, 0.3),
            new RetractArmsToPosition(climber, -0.69),
            new PushArmsForwardToPosition(climber, 0.37, 0.46),
            new LatchOntoBar(climber)
        );
    }
}