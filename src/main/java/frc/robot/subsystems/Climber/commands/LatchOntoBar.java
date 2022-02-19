package frc.robot.subsystems.Climber.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber.Climber;

public class LatchOntoBar extends CommandBase{
    private Climber climber;

    public LatchOntoBar(Climber cb) {
        climber = cb;
        addRequirements(cb);
    }

    @Override
    public void initialize(){
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
