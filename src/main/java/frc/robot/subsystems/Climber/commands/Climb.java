package frc.robot.subsystems.Climber.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber.Climber;

public class Climb extends SequentialCommandGroup{
    private Climber climber;
    
    public Climb(Climber cb){
        climber = cb;
        addRequirements(cb);

        addCommands(
            new PushArmsForward(climber),
            new ExtendArms(climber),
            new PullArmsBack(climber),
            new RetractArms(climber)
        );
    }
}