package frc.robot.subsystems.Climber.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber.Climber;

import java.util.logging.Logger;

public class PullArmsBack extends CommandBase{
    private Climber climber;

    private static Logger logger = Logger.getLogger(PullArmsBack.class.getName());

    public PullArmsBack(Climber cb) {
        climber = cb;
        addRequirements(cb);
    }

    @Override
    public void initialize(){
        logger.info("Pull Arms Back");
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
        return (Math.abs(climber.getRightPotentiometerPosition() - climber.getRightMinBackPosition()) < 0.02 && Math.abs(climber.getLeftPotentiometerPosition() - climber.getLeftMinBackPosition()) < 0.02);
    }
}
