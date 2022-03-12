package frc.robot.subsystems.Elevator.commands;

import java.util.logging.Logger;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Elevator.Elevator;

public class ExtendArmsToPosition extends CommandBase {
    private static Logger logger = Logger.getLogger(ExtendArmsToPosition.class.getName());

    private Elevator elevator;
    private double targetRotations;

    public ExtendArmsToPosition(Elevator el, double rot) {
        elevator = el;
        targetRotations = rot;
        addRequirements(el);
    }

    @Override
    public void initialize(){
        logger.info("Extending Arms");
        elevator.setTargetRotations(targetRotations);
    }

    @Override
    public void execute(){
        logger.info("Moving Arms");
        elevator.setVoltageToWinchMotor();
    }

    @Override
    public void end(boolean interrupted){
        elevator.stopWinch();
        logger.info("Finished Extending Arms");
    }

    @Override
    public boolean isFinished(){
        return elevator.isWinchAtGoal();
    }
}