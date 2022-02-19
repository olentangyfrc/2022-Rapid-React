package frc.robot.subsystems.Climber.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber.Climber;

public class PushArmsForward extends CommandBase{
    private Climber climber;

    public PushArmsForward(Climber cb) {
        climber = cb;
        addRequirements(cb);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        climber.pushArmsForward();
    }

    @Override
    public void end(boolean interrupted){
        climber.stopRightLinearActuator();
        climber.stopLeftLinearActuator();
    }

    @Override
    public boolean isFinished(){
        return (Math.abs(climber.getRightPotentiometerPosition() - climber.getRightMaxForwardPosition()) < 0.2 && Math.abs(climber.getLeftPotentiometerPosition() - climber.getLeftMaxForwardPosition()) < 0.2);
    }
}
