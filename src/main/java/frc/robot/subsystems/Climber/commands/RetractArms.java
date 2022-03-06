package frc.robot.subsystems.Climber.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator.Elevator;

import java.util.logging.Logger;

public class RetractArms extends CommandBase{
    private Elevator elevator;

    private static Logger logger = Logger.getLogger(RetractArms.class.getName());

    public RetractArms(Elevator el) {
        elevator = el;
        addRequirements(el);
    }

    @Override
    public void initialize(){
        logger.info("Retract Arms");
    }

    @Override
    public void execute(){
        elevator.retractArms();
    }

    @Override
    public void end(boolean interrupted){
        elevator.stopWinch();
    }

    @Override
    public boolean isFinished(){
        return Math.abs(elevator.getMinHeight() - elevator.getWinchPosition()) <= 0.1;
    }
}
