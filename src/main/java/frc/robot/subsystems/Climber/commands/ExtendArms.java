package frc.robot.subsystems.Climber.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber.Climber;

import java.util.logging.Logger;

public class ExtendArms extends CommandBase{
    private Climber climber;
    private static Logger logger = Logger.getLogger(ExtendArms.class.getName());

    public ExtendArms(Climber cb) {
        climber = cb;
        addRequirements(cb);
    }

    @Override
    public void initialize(){
        logger.info("Extend Arms");
    }

    @Override
    public void execute(){
        climber.extendArms();
    }

    @Override
    public void end(boolean interrupted){
        climber.stopWinch();
    }

    @Override
    public boolean isFinished(){
        return Math.abs(climber.getMaxHeight() - climber.getWinchPosition()) <= 0.1;
    }
}
