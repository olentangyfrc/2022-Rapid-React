package frc.robot.subsystems.Climber.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber.Climber;

public class NudgeArmsBackwards extends CommandBase{
    public static final double NUDGE_AMOUNT = 0.05;
    private Climber climber;

    public NudgeArmsBackwards(Climber cb) {
      climber = cb;
      addRequirements(cb);
    }
    
    @Override
    public void initialize(){
      double newPosition = MathUtil.clamp(climber.getAverageArmPosition() - NUDGE_AMOUNT, 0, Climber.MAX_ARM_POSITION);
      climber.setTargetArmPosition(newPosition);
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