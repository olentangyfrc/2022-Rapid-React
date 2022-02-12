// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

/**
 * Portman provides a control mechanism for subsystems to acquire ports for their
 * devices. It does it my keeping track of ports that have been allocated and throws
 * an exception if a port is requested more than once. It help to track down invalid port assignments
 * 
 * Usage of this would look something like the following in a hypothetical mecanum:
 * 
 * public class Mecanum {
 * TalonSRX frontRight;
 * 
 * public init() throws Exception {
 *  frontRight  = new TalonSRX(portMan.acquirePort(PortMan.PortType.CAN, 3, "Mecanum.frontRight"));
 *
 */
public class PortManager {

  // Assignment arrays. These will store the names of the devices assigned to each port.
  private String[] digital = new String[21];
  private String[] analog = new String[6];
  private String[] relay = new String[4];
  private String[] pwm = new String[10];
  private String[] pcm = new String[8];
  private String[] can = new String[63];

  /**
   * If the port is available, store the assignment and return the given port.
   * If the port is unavailable, throw an exception.
   * 
   * @param portType The type of port to assign to
   * @param port The port to assign
   * @param deviceName The name of the device being assigned.
   * @return The given port if it's available
   * @throws Exception If the given port is unavailable or an invalid port is given.
   */
  public int aquirePort(PortType portType, int port, String deviceName) throws Exception {
    String[] portArray;
    // Select the correct assignment array
    switch(portType) {
      case DIGITAL:
        portArray = digital;
        break;
      case ANALOG:
        portArray = analog;
        break;
      case RELAY:
        portArray = relay;
        break;
      case PWM:
        portArray = pwm;
        break;
      case PCM:
        portArray = pcm;
        break;
      case CAN:
        portArray = can;
        break;
      default:
        throw new Exception("PortType [" + portType.toString() + "] has not been implemented.");
    }

    // Check for invalid port
    if(port < 0 || port >= portArray.length) {
      throw new Exception("Port [" + port + "] is invalid for PortType [" + portType.toString() + "]");
    }

    // Check if port has already been assigned
    if(portArray[port] != null) {
      throw new Exception("[" + portType.toString() + " " + port + "] has already been assigned to device [" + portArray[port] + "]");
    } else {
      portArray[port] = deviceName;
    }

    return port;
  }

  /**
   * Types of ports on the robot
   */
  public enum PortType {
    DIGITAL,
    ANALOG,
    RELAY,
    PWM,
    PCM,
    CAN
  }
}
