package frc.robot.subsystems.Climber.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber.Climber;

import java.util.logging.Logger;

public class PushArmsForwardToPosition extends CommandBase{
    private Climber climber;

    private static Logger logger = Logger.getLogger(PushArmsForward.class.getName());

    private double position;

    public PushArmsForwardToPosition(Climber cb, double pos) {
        climber = cb;
        addRequirements(cb);
        position = MathUtil.clamp(pos, 0, climber.MAX_ARM_POSITION);
    }

    @Override
    public void initialize(){
        logger.info("Pushing Arms Forward");
        climber.setTargetArmPosition(position);
    }

    @Override
    public void execute(){
        climber.setArmVoltage();
    }

    @Override
    public void end(boolean interrupted){
        climber.stopRightLinearActuator();
        climber.stopLeftLinearActuator();
    }

    @Override
    public boolean isFinished(){
        return climber.armsAtPosition();
    }
}