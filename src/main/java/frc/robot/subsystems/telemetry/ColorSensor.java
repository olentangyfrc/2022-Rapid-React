package frc.robot.subsystems.telemetry;

import java.util.Map;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.util.Color;

public class ColorSensor {
    
    ColorSensorV3 colorSensor;
    SimpleWidget colorWidget;
    NetworkTableEntry color;

    /**
     * Initializes ColorSensorV3 object
     */
    public ColorSensor() {
        colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

        colorWidget = Shuffleboard.getTab("Competition").add("Color", true).withWidget(BuiltInWidgets.kBooleanBox);
        
        color = colorWidget.getEntry();
    }

    /**
     * Updated the ball color shuffleboard entry. THIS MUST BE CALLED PERIODICALLY
     */
    public void periodic() {
        color.forceSetBoolean(true);
        switch (getColor()) {
            case red:
                colorWidget.withProperties(Map.of("colorWhenTrue", "red"));
                break;
            case blue:
                colorWidget.withProperties(Map.of("colorWhenTrue", "blue"));
                break;
            default:
                colorWidget.withProperties(Map.of("colorWhenTrue", "white"));
                break;
        }
    }

    /**
     * @return The ballColor based on the color the sensor is detecting
     */
    private ballColor getColor() {
        Color colorBall = colorSensor.getColor();
        System.out.println("R: " + colorBall.red + " G: " + colorBall.green + " B: " + colorBall.blue);
        if (colorBall.green > 0.4 && colorBall.blue > 0.34) return ballColor.blue;
        else if (colorBall.red > 0.55) return ballColor.red;
        else return ballColor.none;
    }

    /**
     * Enum for valid ball colors
     */
    private enum ballColor {
        red,
        blue,
        none
    }

}
