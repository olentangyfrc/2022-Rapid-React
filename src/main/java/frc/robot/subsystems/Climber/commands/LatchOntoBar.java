package frc.robot.subsystems.Climber.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber.Climber;

public class LatchOntoBar extends CommandBase{
    private Climber climber;
    private boolean stop;

    public LatchOntoBar(Climber cb) {
        climber = cb;
        addRequirements(cb);
    }

    @Override
    public void initialize(){
        stop = false;
    }

    @Override
    public void execute(){
        climber.latchOntoBar();
    }

    @Override
    public void end(boolean interrupted){
        stop = true;
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
