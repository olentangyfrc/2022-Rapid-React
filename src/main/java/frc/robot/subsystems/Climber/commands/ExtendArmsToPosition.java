package frc.robot.subsystems.Climber.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Elevator.Elevator;

import java.util.logging.Logger;

public class ExtendArmsToPosition extends CommandBase{
    private Elevator elevator;
    private static Logger logger = Logger.getLogger(ExtendArms.class.getName());
    private double rotations;
    private final double MAX_ACCELERATION = 0.1;
    
    private TrapezoidProfile.Constraints constraints;
    private TrapezoidProfile.State goal;
    private TrapezoidProfile.State setpoint;
    private boolean isBotOnBar;

    private static double kDt = 0.02;

    public ExtendArmsToPosition(Elevator el, double rot, boolean onBar) {
        elevator = el;
        addRequirements(el);
        goal = new TrapezoidProfile.State(rot, 0);
        isBotOnBar = onBar;

        if(onBar == true){
            constraints = new TrapezoidProfile.Constraints(elevator.getMaxSpeedOnBar(MAX_ACCELERATION), MAX_ACCELERATION);
        }
        else{
            constraints = new TrapezoidProfile.Constraints(elevator.getMaxSpeedOffBar(MAX_ACCELERATION), MAX_ACCELERATION);
        }
    }

    @Override
    public void initialize(){
        logger.info("Extending Arms");
    }

    private NetworkTableEntry entry = Shuffleboard.getTab("Climber").add("Target Velocity", 0).getEntry();

    @Override
    public void execute(){
        // Create a motion profile with the given maximum velocity and maximum
        // acceleration constraints for the next setpoint, the desired goal, and the
        // current setpoint.
        var profile = new TrapezoidProfile(constraints, goal, elevator.getState());
    
        // Retrieve the profiled setpoint for the next timestep. This setpoint moves
        // toward the goal while obeying the constraints.
        var setpoint = profile.calculate(kDt);
        entry.setNumber(setpoint.velocity);
        if(isBotOnBar == true){
            elevator.setVelocityOnWinchMotorOnBar(setpoint.velocity);
        }
        else{
            elevator.setVelocityOnWinchMotorOffBar(setpoint.velocity);
        }

        //logger.info("Total Time:" + profile.totalTime());
    }

    @Override
    public void end(boolean interrupted){
        elevator.stopWinch();
    }

    @Override
    public boolean isFinished(){
        return new TrapezoidProfile(constraints, goal, elevator.getState()).isFinished(kDt);
    }

    public double getTargetVelocity(){
        return setpoint.velocity;
    }
}
