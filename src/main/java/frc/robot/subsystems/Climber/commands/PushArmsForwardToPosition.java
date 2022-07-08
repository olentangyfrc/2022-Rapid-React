package frc.robot.subsystems.Climber.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber.Climber;

import java.util.logging.Logger;

/**
 * Set the arms' target position to a new value and the move to that new target.
 */
public class PushArmsForwardToPosition extends CommandBase{
    private Climber climber;

    private double position; // The target position

    public PushArmsForwardToPosition(Climber cb, double pos) {
        climber = cb;
        addRequirements(cb);
        // Make sure we don't try to move somewhere we can't
        position = MathUtil.clamp(pos, 0, Climber.MAX_ARM_POSITION);
    }

    @Override
    public void initialize(){
        climber.setTargetArmPosition(position);
    }

    @Override
    public void execute(){
        climber.applyArmVoltage();
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