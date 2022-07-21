package frc.robot.subsystems.Elevator.commands;

import java.util.logging.Logger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator.Elevator;

/**
 * Extend or retract the climbing arms vertically to a given position in rotations of the gear.
 */
public class ExtendArmsToPosition extends CommandBase {
    private static Logger logger = Logger.getLogger(ExtendArmsToPosition.class.getName());

    private Elevator elevator;
    // Target position in rotations of the gear
    private double targetRotations;

    /**
     * Construct a new ExtendArmsToPosition command
     * 
     * @param el The elevator subsystem to move
     * @param rot The target position in rotations of the gear.
     */
    public ExtendArmsToPosition(Elevator el, double rot) {
        elevator = el;
        targetRotations = rot;
        addRequirements(el);
    }

    @Override
    public void initialize(){
        elevator.setTargetRotations(targetRotations);
    }

    /**
     * This command ends when the elevator position is within tolerance of the target
     */
    @Override
    public boolean isFinished(){
        return elevator.isWinchAtGoal();
    }
}