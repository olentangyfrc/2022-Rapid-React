package frc.robot.subsystems.Climber.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber.Climber;

import java.util.logging.Logger;

public class PushArmsForward extends CommandBase{
    private Climber climber;

    private static Logger logger = Logger.getLogger(PushArmsForward.class.getName());

    public PushArmsForward(Climber cb) {
        climber = cb;
        addRequirements(cb);
    }

    @Override
    public void initialize(){
        logger.info("Push Arms Forward");
    }

    @Override
    public void execute(){
        climber.pushArmsForwardWithPercent();
    }

    @Override
    public void end(boolean interrupted){
        climber.stopRightLinearActuator();
        climber.stopLeftLinearActuator();
    }

    @Override
    public boolean isFinished(){
        return (Math.abs(climber.getRightPotentiometerPosition() - climber.getRightMaxForwardPosition()) < 0.01 && Math.abs(climber.getLeftPotentiometerPosition() - climber.getLeftMaxForwardPosition()) < 0.01);
    }
}
