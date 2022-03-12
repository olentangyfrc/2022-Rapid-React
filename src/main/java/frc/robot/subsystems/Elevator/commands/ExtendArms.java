package frc.robot.subsystems.Elevator.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator.Elevator;

import java.util.logging.Logger;

public class ExtendArms extends CommandBase{
    private Elevator elevator;

    private static Logger logger = Logger.getLogger(RetractArms.class.getName());

    public ExtendArms(Elevator el) {
        elevator = el;
        addRequirements(el);
    }

    @Override
    public void initialize(){
        logger.info("Extend Arms");
    }

    @Override
    public void execute(){
        elevator.extendArms();
    }

    @Override
    public void end(boolean interrupted){
        elevator.stopWinch();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
