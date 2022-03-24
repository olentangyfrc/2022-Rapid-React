package frc.robot.subsystems.Elevator.commands;

import java.util.logging.Logger;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Elevator.Elevator;

/**
 * Nudge arms down. For use with operator control to make adjustments.
 */
public class NudgeArmsDown extends InstantCommand {
    private Elevator elevator;

    private static Logger logger = Logger.getLogger(NudgeArmsDown.class.getName());

    public NudgeArmsDown(Elevator el) {
        elevator = el;
    }

    @Override
    public void initialize(){
        logger.info("Extend Arms");
        elevator.setTargetRotations(elevator.getTargetRotations() - 0.2);
    }
}
