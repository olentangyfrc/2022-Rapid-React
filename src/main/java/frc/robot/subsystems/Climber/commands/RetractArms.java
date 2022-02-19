package frc.robot.subsystems.Climber.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber.Climber;

public class RetractArms extends CommandBase{
    private Climber climber;
    private boolean stop;

    public RetractArms(Climber cb) {
        climber = cb;
        addRequirements(cb);
    }

    @Override
    public void initialize(){
        stop = false;
    }

    @Override
    public void execute(){
        climber.retractArms();
    }

    @Override
    public void end(boolean interrupted){
        stop = true;
    }

    @Override
    public boolean isFinished(){
        if(Math.abs(climber.getMinHeight() - climber.getWinchPosition()) <= 0.1) {
            climber.stopWinch();
            return true;
        }
        return false;
    }
}
