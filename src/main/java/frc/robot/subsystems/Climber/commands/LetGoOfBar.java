package frc.robot.subsystems.Climber.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber.Climber;

import java.util.logging.Logger;

public class LetGoOfBar extends CommandBase{
    private Climber climber;

    private static Logger logger = Logger.getLogger(LetGoOfBar.class.getName());

    public LetGoOfBar(Climber cb){
        climber = cb;
        addRequirements(cb);
    }

    @Override
    public void initialize(){
        logger.info("Let Go Of Bar");
    }

    @Override
    public void execute(){
        climber.letGoOfBar();
    }

    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
