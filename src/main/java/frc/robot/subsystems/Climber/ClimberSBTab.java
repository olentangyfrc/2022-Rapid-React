package frc.robot.subsystems.Climber;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.commands.ExtendArmsToPosition;

/**
 * This manages the shuffleboard tab for the climber subsystem
 */
public class ClimberSBTab {
    public Climber climber;
    public Elevator elevator;
    public ShuffleboardTab tab;
    public ExtendArmsToPosition arms;

    public NetworkTableEntry rightPotentiometerPosition;
    public NetworkTableEntry leftPotentiometerPosition;
    public NetworkTableEntry winchPercentOutput;
    public NetworkTableEntry percentActuatorLength;
    public NetworkTableEntry positionActuatorLength;

    public ClimberSBTab (Climber cb, Elevator el){
        climber = cb;
        elevator = el;
        tab = Shuffleboard.getTab("Climber");

        tab.addNumber("Right Potentiometer Position", climber::getRightPotentiometerPosition);
        tab.addNumber("Left Potentiometer Position", climber::getLeftPotentiometerPosition);
        tab.addNumber("Winch Encoder Position in Rotations", elevator::getPosition);
        tab.addNumber("Current Velocity", elevator::getVelocity);
    }

    public void update(){
    }
}