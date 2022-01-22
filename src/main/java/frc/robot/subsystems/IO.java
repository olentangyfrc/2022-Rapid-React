// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.interfaces.OzoneSubsystem;

/** Add your docs here. */
public class IO extends OzoneSubsystem {
    private InputMethod inputMethod;

    private XboxController xbox;

    private Joystick leftStick;
    private Joystick rightStick;

    @Override
    public void init() {
        inputMethod = determineInputMethod();
        
        PortManager portManager = SubsystemFactory.getInstance().getPortManager();

        if(inputMethod == InputMethod.XBOX) {
            xbox = new XboxController(0);
        } else {
            leftStick = new Joystick(0);
            rightStick = new Joystick(1);
        }
    }

    /**
     * Get the strafe input for the bot.
     * 
     * @return the strafe input
     */
    public double getStrafe() {
        if(inputMethod == InputMethod.XBOX) {
            return curveInput(xbox.getLeftX());
        } else {
            return curveInput(leftStick.getX());
        }
    }

    /**
     * Get the forward input for the bot.
     * 
     * @return the forward input
     */
    public double getForward() {
        if(inputMethod == InputMethod.XBOX) {
            return curveInput(xbox.getLeftY());
        } else {
            return curveInput(leftStick.getY());
        }
    }

    /**
     * Get the rotation input for the bot.
     * 
     * @return the rotation input
     */
    public double getRotation() {
        if(inputMethod == InputMethod.XBOX) {
            return curveInput(xbox.getRightX());
        } else {
            return curveInput(rightStick.getX());
        }
    }

    /**
     * Apply an input curve to the input. This will square the input to make it feel more natural for drivers.
     * 
     * @param input the original input
     * @return the curved input
     */
    public double curveInput(double input) {
        return Math.copySign(Math.pow(input, 2), input);
    }

    /**
     * Return the input method connected to the driverstation, xbox or joysticks
     * 
     * @return the input method
     */
    public InputMethod determineInputMethod() {
        if(DriverStation.getJoystickIsXbox(0)) {
            return InputMethod.XBOX;
        } else {
            return InputMethod.JOYSTICKS;
        }
    }
    
    /**
     * The method of input connected to the driverstation.
     */
    private enum InputMethod {
        XBOX, // An xbox controller on port 0
        JOYSTICKS // Two joysticks, left connected to port 0 and right to port 1
    }
}
