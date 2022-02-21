package frc.robot.subsystems.Climber.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber.Climber;

import java.util.logging.Logger;

public class RetractArms extends CommandBase{
    private Climber climber;

    private static Logger logger = Logger.getLogger(RetractArms.class.getName());

    public RetractArms(Climber cb) {
        climber = cb;
        addRequirements(cb);
    }

    @Override
    public void initialize(){
        logger.info("Retract Arms");
    }

    @Override
    public void execute(){
        climber.retractArms();
    }

    @Override
    public void end(boolean interrupted){
        climber.stopWinch();
    }

    @Override
    public boolean isFinished(){
        return Math.abs(climber.getMinHeight() - climber.getWinchPosition()) <= 0.1;
    }
}
