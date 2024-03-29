package frc.robot.subsystems.Climber.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber.Climber;

import java.util.logging.Logger;

public class LatchOntoBar extends CommandBase{
    private Climber climber;

    private static Logger logger = Logger.getLogger(LatchOntoBar.class.getName());

    public LatchOntoBar(Climber cb) {
        climber = cb;
        addRequirements(cb);
    }

    @Override
    public void initialize(){
        logger.info("Latch Onto Bar");
    }

    @Override
    public void execute(){
        climber.latchOntoBar();
    }

    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
