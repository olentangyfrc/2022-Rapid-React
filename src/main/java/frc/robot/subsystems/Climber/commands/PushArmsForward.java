package frc.robot.subsystems.Climber.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber.Climber;

public class PushArmsForward extends CommandBase{
    private Climber climber;
    private boolean stop;

    public PushArmsForward(Climber cb) {
        climber = cb;
        addRequirements(cb);
    }

    @Override
    public void initialize(){
        stop = false;
    }

    @Override
    public void execute(){
        climber.pushArmsForward();
    }

    @Override
    public void end(boolean interrupted){
        stop = true;
    }

    @Override
    public boolean isFinished(){
        if(Math.abs(climber.getRightPotentiometerPosition() - climber.getMaxForwardPosition()) < 0.2){
            climber.stopRightLinearActuator();
            climber.stopLeftLinearActuator();
            return true;
        }
        return false;
    }
}
