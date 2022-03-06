package frc.robot.subsystems.Climber;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.Climber.commands.ExtendArmsToPosition;
import frc.robot.subsystems.Elevator.Elevator;

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

        //rightPotentiometerPosition = tab.add("Right Potentiometer Position", 0.0).getEntry();
        //leftPotentiometerPosition = tab.add("Left Potentiometer Position", 0.0).getEntry();

        tab.addNumber("Right Potentiometer Position", climber::getRightPotentiometerPosition);
        tab.addNumber("Left Potentiometer Position", climber::getLeftPotentiometerPosition);
        tab.addNumber("Max Right Potentiometer Position", climber::getRightMaxForwardPosition);
        tab.addNumber("Max Left Potentiometer Position", climber::getLeftMaxForwardPosition);
        tab.addNumber("Min Right Potentiometer Position", climber::getRightMinBackPosition);
        tab.addNumber("Min Left Potentiometer Position", climber::getLeftMinBackPosition);
        tab.addNumber("Winch Encoder Position in Rotations", elevator::getEncoderRotations);
        tab.addNumber("Winch Encoder Position in Encoder Ticks", elevator::getEncoderPosition);
        tab.addNumber("Current Velocity", elevator::getVelocity);

        //winchPercentOutput = tab.add("Set Winch Percent Output (-1 to 1)", 0.05).getEntry();
        //percentActuatorLength = tab.add("Set Actuator Length Target with Percentage", 0.0).getEntry();
        //positionActuatorLength = tab.add("Actuator Length Target in Position", 0.0).getEntry();
    }

    public void update(){
        //rightPotentiometerPosition.setDouble(climber.getRightPotentiometerPosition());
        //leftPotentiometerPosition.setDouble(climber.getLeftPotentiometerPosition());
        //climber.setVerticalPercentOutput(winchPercentOutput.getDouble(0.0));
        //climber.setLinearActuatorLengthInPercent(percentActuatorLength.getDouble(0.0));
    }
}
