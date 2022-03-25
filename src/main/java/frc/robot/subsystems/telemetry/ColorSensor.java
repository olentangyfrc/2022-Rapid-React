package frc.robot.subsystems.telemetry;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.util.Color;

public class ColorSensor {
    
    ColorSensorV3 colorSensor;

    public ColorSensor() {
        colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    }

    public void periodic() {
        switch (getColor()) {
            case red:
                Shuffleboard.getTab("Competition").add("Ball Color", Color.kRed);
                break;
            case blue:
                Shuffleboard.getTab("Competition").add("Ball Color", Color.kBlue);
                break;
            default:
                Shuffleboard.getTab("Competition").add("Ball Color", Color.kWhite);
                break;
        }
    }

    private ballColor getColor() {
        if (colorSensor.getColor().equals(Color.kBlue)) return ballColor.blue;
        else if (colorSensor.getColor().equals(Color.kRed)) return ballColor.red;
        else return null;
    }

    private enum ballColor {
        red,
        blue
    }

}
