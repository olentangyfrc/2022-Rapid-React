package frc.robot.subsystems.Climber.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber.Climber;

import java.util.logging.Logger;

public class RetractArmsToPosition extends CommandBase{
    private Climber climber;

    private static Logger logger = Logger.getLogger(RetractArms.class.getName());

    private double position;

    public RetractArmsToPosition(Climber cb, double pos) {
        climber = cb;
        addRequirements(cb);
        position = pos;
    }

    @Override
    public void initialize(){
        logger.info("Retracting Arms");
    }

    @Override
    public void execute(){
        //climber.retractArms();
    }

    @Override
    public void end(boolean interrupted){
        //climber.stopWinch();
    }

    @Override
    public boolean isFinished(){
        //return Math.abs(position - climber.getWinchPosition()) <= 0.2;
        return true;
    }
}
