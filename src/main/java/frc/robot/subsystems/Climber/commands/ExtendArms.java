package frc.robot.subsystems.Climber.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber.Climber;

public class ExtendArms extends CommandBase{
    private Climber climber;
    private boolean stop;

    public ExtendArms(Climber cb) {
        climber = cb;
        addRequirements(cb);
    }

    @Override
    public void initialize(){
        stop = false;
    }

    @Override
    public void execute(){
        climber.extendArms();
    }

    @Override
    public void end(boolean interrupted){
        stop = true;
    }

    @Override
    public boolean isFinished(){
        if(climber.getMaxHeight() == climber.getWinchPosition()) {
            return true;
        }
        return false;
    }
}
