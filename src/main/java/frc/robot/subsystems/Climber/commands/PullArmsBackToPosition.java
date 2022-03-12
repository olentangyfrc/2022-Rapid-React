package frc.robot.subsystems.Climber.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber.Climber;

import java.util.logging.Logger;

public class PullArmsBackToPosition extends CommandBase{
    private Climber climber;

    private static Logger logger = Logger.getLogger(PushArmsForward.class.getName());

    private double leftPosition;
    private double rightPosition;

    public PullArmsBackToPosition(Climber cb, double leftPos, double rightPos) {
        climber = cb;
        addRequirements(cb);
        leftPosition = leftPos;
        rightPosition = rightPos;
    }

    @Override
    public void initialize(){
        logger.info("Pulling Arms Backward");
    }

    @Override
    public void execute(){
        climber.pullArmsBackWithPercent();
    }

    @Override
    public void end(boolean interrupted){
        climber.stopRightLinearActuator();
        climber.stopLeftLinearActuator();
    }

    @Override
    public boolean isFinished(){
        return ((Math.abs(climber.getRightPotentiometerPosition() - climber.getRightMinBackPosition()) < 0.1 && Math.abs(climber.getLeftPotentiometerPosition() - climber.getLeftMinBackPosition()) < 0.1) || (Math.abs(rightPosition - climber.getRightPotentiometerPosition()) < 0.1 && Math.abs(leftPosition - climber.getLeftPotentiometerPosition()) < 0.1));
    }
}
