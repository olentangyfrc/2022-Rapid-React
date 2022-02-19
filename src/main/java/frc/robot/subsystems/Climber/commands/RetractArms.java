package frc.robot.subsystems.Climber.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber.Climber;

public class RetractArms extends CommandBase{
    private Climber climber;

    public RetractArms(Climber cb) {
        climber = cb;
        addRequirements(cb);
    }

    @Override
    public void initialize(){
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
