// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.logging.Logger;

// WPI imports:
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick.ButtonType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.SubsystemFactory.BotType;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.telemetry.commands.ZeroGyro;

// Project imports

/**
 * This subsytem handles all of the user input and output of the robot.
 * 
 * TODO: Add button binding functionality
 */
public class IO extends SubsystemBase {
    public static final int XBOX_PORT = 0;
    public static final int LEFT_STICK_PORT = 1;
    public static final int RIGHT_STICK_PORT = 2;
    public static final double DEADZONE = 0.09;

    private HashMap<Binding, Command> buttonBindings = new HashMap<Binding, Command>();

    private InputMethod inputMethod;

    private XboxController xbox;

    private Joystick leftStick;
    private Joystick rightStick;

    private Logger logger = Logger.getLogger("IO");

    public void init(BotType bot) throws Exception{
        inputMethod = determineInputMethod();

        if(inputMethod.equals(InputMethod.XBOX)) {
            xbox = new XboxController(XBOX_PORT);
        } else {
            leftStick = new Joystick(LEFT_STICK_PORT);
            rightStick = new Joystick(RIGHT_STICK_PORT);
        }
    }

    /**
     * Get the strafe input for the bot.
     * 
     * @return the strafe input
     */
    public double getStrafe() {
        if(inputMethod.equals(InputMethod.XBOX)) {
            return filterInput(xbox.getLeftX());
        } else {
            return filterInput(leftStick.getX());
        }
    }

    /**
     * Get the forward input for the bot.
     * 
     * @return the forward input
     */
    public double getForward() {
        if(inputMethod.equals(InputMethod.XBOX)) {
            return filterInput(-xbox.getLeftY());
        } else {
            return filterInput(-leftStick.getY());
        }
    }

    /**
     * Get the rotation input for the bot.
     * 
     * @return the rotation input
     */
    public double getRotation() {
        if(inputMethod.equals(InputMethod.XBOX)) {
            return filterInput(xbox.getRightX());
        } else {
            return filterInput(rightStick.getX());
        }
    }

    /**
     * Apply an input curve to the input. This will square the input to make it feel more natural for drivers.
     * <p>
     * Also apply the joystick deadzone.
     * 
     * @param input the original input
     * @return the curved input
     */
    public double filterInput(double input) {
        if(Math.abs(input) <= DEADZONE) {
            input = 0;
        }
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

    public void bind(Command command, Button xboxButton, StickButton stickButton, ButtonActionType type) throws Exception {
        GenericHID joystick;
        int button;

        // Determine which joystick to bind to
        if(inputMethod == InputMethod.XBOX) {
            joystick = xbox;
            button = xboxButton.value;
        } else {
            if(stickButton.ordinal() <= 10) {
                joystick = leftStick;
                button = stickButton.ordinal() + 1;
            } else {
                joystick = rightStick;
                button = stickButton.ordinal() - 10 + 1;
            }
        }

        Binding binding = new Binding(joystick, button, type);

        // Check if there is already a similar binding.
        if(buttonBindings.containsKey(binding)) {
            logger.severe("Failed to assign: " + binding.toString() + " is already assigned to Command [" + buttonBindings.get(binding).getName() + "]");
            return;
        }
        
        buttonBindings.put(binding, command);

        JoystickButton jButton = new JoystickButton(joystick, button);

        switch(type) {
            case CANCEL_WHEN_PRESSED:
                jButton.cancelWhenPressed(command);
                break;
            case TOGGLE_WHEN_PRESSED:
                jButton.toggleWhenPressed(command);
                break;
            case WHEN_HELD:
                jButton.whenHeld(command);
                break;
            case WHEN_PRESSED:
                jButton.whenPressed(command);
                break;
            case WHEN_RELEASED:
                jButton.whenReleased(command);
                break;
            case WHILE_HELD:
                jButton.whileHeld(command);
                break;
        }
    }
    
    /**
     * The method of input connected to the driverstation.
     */
    private enum InputMethod {
        XBOX, // An xbox controller on port 0
        JOYSTICKS // Two joysticks, left connected to port 0 and right to port 1
    }

    /**
     * The joystick buttons
     */
    public enum StickButton {
        LEFT_1,
        LEFT_2,
        LEFT_3,
        LEFT_4,
        LEFT_5,
        LEFT_6,
        LEFT_7,
        LEFT_8,
        LEFT_9,
        LEFT_10,
        LEFT_11,
        RIGHT_1,
        RIGHT_2,
        RIGHT_3,
        RIGHT_4,
        RIGHT_5,
        RIGHT_6,
        RIGHT_7,
        RIGHT_8,
        RIGHT_9,
        RIGHT_10,
        RIGHT_11,
    }

    /**
     * A type of button assignment.
     */
    public enum ButtonActionType {
        CANCEL_WHEN_PRESSED,
        TOGGLE_WHEN_PRESSED,
        WHEN_HELD,
        WHEN_PRESSED,
        WHEN_RELEASED,
        WHILE_HELD
    }

    /**
     * Represent a button binding with a joystick, button, and type
     */
    private static class Binding {
        private GenericHID joystick;
        private int button;
        private ButtonActionType type;

        public Binding(GenericHID joystick, int button, ButtonActionType type) {
            this.joystick = joystick;
            this.button = button;
            this.type = type;
        }

        public GenericHID getJoystick() {
            return joystick;
        }

        public void setJoystick(GenericHID joystick) {
            this.joystick = joystick;
        }

        public int getButton() {
            return button;
        }

        public void setButton(int button) {
            this.button = button;
        }

        public ButtonActionType getType() {
            return type;
        }

        public void setType(ButtonActionType type) {
            this.type = type;
        }

        @Override
        public String toString() {
            return "Binding [button=" + button + ", joystick=" + joystick + ", type=" + type + "]";
        }

        @Override
        public int hashCode() {
            final int prime = 31;
            int result = 1;
            result = prime * result + button;
            result = prime * result + ((joystick == null) ? 0 : joystick.hashCode());
            result = prime * result + ((type == null) ? 0 : type.hashCode());
            return result;
        }

        @Override
        public boolean equals(Object obj) {
            if (this == obj)
                return true;
            if (obj == null)
                return false;
            if (getClass() != obj.getClass())
                return false;
            Binding other = (Binding) obj;
            if (button != other.button)
                return false;
            if (joystick == null) {
                if (other.joystick != null)
                    return false;
            } else if (!joystick.equals(other.joystick))
                return false;
            if (type != other.type)
                return false;
            return true;
        }

                
    }
}
