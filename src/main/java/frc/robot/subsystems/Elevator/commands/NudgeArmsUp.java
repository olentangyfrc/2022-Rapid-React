package frc.robot.subsystems.Elevator.commands;

import java.util.logging.Logger;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Elevator.Elevator;

/**
 * Nudge arms up. For use with operator control to make adjustments.
 */
public class NudgeArmsUp extends InstantCommand{
    // The amount of rotations to decrease the target position of the elevator by.
    public static final double NUDGE_AMOUNT = 0.5;

    private Elevator elevator;

    private static Logger logger = Logger.getLogger(NudgeArmsDown.class.getName());

    /**
     * Construct a new NudgeArmsUp command
     * 
     * @param el The elevator subsystem to move
     */
    public NudgeArmsUp(Elevator el) {
        elevator = el;
    }

    @Override
    public void initialize() {
        // Decrease the target position by a small amount to retract arms.
        elevator.setTargetRotations(elevator.getTargetRotations() + NUDGE_AMOUNT);
    }
}
