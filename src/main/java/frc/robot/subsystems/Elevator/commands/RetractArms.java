package frc.robot.subsystems.Elevator.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Elevator.Elevator;

import java.util.logging.Logger;

public class RetractArms extends InstantCommand {
    private Elevator elevator;

    private static Logger logger = Logger.getLogger(RetractArms.class.getName());

    public RetractArms(Elevator el) {
        elevator = el;
    }

    @Override
    public void initialize(){
        logger.info("Extend Arms");
        elevator.setTargetRotations(elevator.getTargetRotations() - 0.2);
    }
}
