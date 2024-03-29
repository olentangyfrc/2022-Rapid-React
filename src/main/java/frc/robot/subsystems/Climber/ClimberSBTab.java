package frc.robot.subsystems.Climber;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.commands.ExtendArmsToPosition;

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
        tab.addNumber("Max Right Potentiometer Position", climber::getRightMaxForwardPosition);
        tab.addNumber("Max Left Potentiometer Position", climber::getLeftMaxForwardPosition);
        tab.addNumber("Min Right Potentiometer Position", climber::getRightMinBackPosition);
        tab.addNumber("Min Left Potentiometer Position", climber::getLeftMinBackPosition);
        tab.addNumber("Winch Encoder Position in Rotations", elevator::getPosition);
        tab.addNumber("Current Velocity", elevator::getVelocity);
    }

    public void update(){
    }
}