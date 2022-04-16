// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


/*
*************************** 
Things need to Updated:
- Gyro Angle
- Location of Camera on Bot
- Send Position
***************************
*/

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.TimesliceRobot;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;





public class networkTables extends SubsystemBase {

  private SwerveDriveOdometry odometry;
  private static double laststabletime = Timer.getFPGATimestamp();
  private Pose2d last_visionmeasurement = new Pose2d();
  private ArrayList<past_object> past_positions = new ArrayList<past_object>(100);

  private int fieldlength = 16;
  private int fieldwidth = 8;

  NetworkTableInstance inst;
  Gyro gyro;
  NetworkTableEntry Time, XEntry, YEntry, ZEntry;

  double lastVisionTime = Timer.getFPGATimestamp();

  public class past_object{
    private Pose2d estimate;
    private double timestamp;
    private boolean isMoving;

    public past_object(Pose2d estimate, double timestamp, boolean isMoving){
      this.estimate = estimate;
      this.timestamp = timestamp;
      this.isMoving = isMoving;
    }
    public Pose2d getEstimatedPosition(){
      return estimate;
    }
    
    public double getFPGATimestamp(){
      return timestamp;
    }

    public boolean getIsMoving() {
      return isMoving;
    }
  }




  public networkTables() {
    odometry = SubsystemFactory.getInstance().getDrivetrain().getSwerveDriveOdometry(); 
    inst = NetworkTableInstance.getDefault();
    gyro = SubsystemFactory.getInstance().getTelemetry().getGyro();
    inst.startClientTeam(4611);
    NetworkTable Tvecs = inst.getTable("SmartDashboard"); //delcares the networktables to the already intizialized instance

    Time = Tvecs.getEntry("Time");
    XEntry = Tvecs.getEntry("X");
    YEntry = Tvecs.getEntry("Y");
    ZEntry = Tvecs.getEntry("Z");

    Time.addListener(event -> {onVisionUpdate(event);}, EntryListenerFlags.kUpdate);
   
  }

  
  public void onVisionUpdate(EntryNotification event ) {
    
    //System.out.println("Time changed value: " + event.getEntry().getValue()); //If time is changed, then it records x,y,z
    double elapsedtime = event.getEntry().getDouble(0);
    //System.out.println("X changed value: " + XEntry.getValue());
    double x = XEntry.getDouble(0);
    //System.out.println("Y changed value: " + YEntry.getValue());
    double y = YEntry.getDouble(0);
    //System.out.println("Z changed value: " + ZEntry.getValue());
    double z = ZEntry.getDouble(0);
   
    updatePositionwithVision(x, y, z, elapsedtime); //converts the x,y,z into final position vector

    // Do work here like updates odometry...
    //System.out.print(position);

  }



  /**
       * Corrects Odermetry with Vision
       * 
       * @param x, @param y, @param z takes in the Tvec from Nano and converst between coordinate system
       * 
       * @param timeStampSeconds is when the vision measurement was made
       */
  public void updatePositionwithVision(double x, double y, double z, double elapsedtime){
    // Build out input vector
    var in_vec = VecBuilder.fill(x, y, z);

    var cv2_correction_mat = Matrix.mat(Nat.N3(), Nat.N3()).fill(
      0, 0, 1,
      -1, 0, 0,
      0, -1, 0);

    var corrected_vec = cv2_correction_mat.times(in_vec);

    //SmartDashboard.putNumber("in_vecx", in_vec.get(0, 0));
    //SmartDashboard.putNumber("in_vecy", in_vec.get(1, 0));
   // SmartDashboard.putNumber("in_vecz", in_vec.get(2, 0));
    


    // Build our rotation matrix
    double pitch = -35 * Math.PI / 180;
    double c = Math.cos(pitch);
    double s = Math.sin(pitch);
    var camera_to_bot = Matrix.mat(Nat.N3(), Nat.N3()).fill(
        c, 0, s,
        0, 1, 0,
      -s, 0, c);

    // Rotatea
    var corrected_bot_oriented = camera_to_bot.times(corrected_vec);
    //SmartDashboard.putNumber("corrected_bot_orientedx", corrected_bot_oriented.get(0, 0));
    // SmartDashboard.putNumber("corrected_bot_orientedy", corrected_bot_oriented.get(1, 0));
    // SmartDashboard.putNumber("corrected_bot_orientedz", corrected_bot_oriented.get(2, 0));



    var trans2_vec = VecBuilder.fill(-0.1, -0.315, 0.76); //Change this where we know the displacement of the camera to the center of the robot
    
    var out_vec = trans2_vec.plus(corrected_bot_oriented);

    SmartDashboard.putNumber("out_vecx", out_vec.get(0, 0));
    SmartDashboard.putNumber("out_vecy", out_vec.get(1, 0));
    SmartDashboard.putNumber("out_vecz", out_vec.get(2, 0));

    

    
    
   
    double gyro_angle = getPastPose(elapsedtime).getEstimatedPosition().getRotation().getRadians();
    c = Math.cos(gyro_angle);
    s = Math.sin(gyro_angle);
    var bot_to_field = Matrix.mat(Nat.N3(), Nat.N3()).fill(
        c, -s, 0,
        s, c, 0,
        0, 0, 1);
    var final_vec = bot_to_field.times(out_vec);
    SmartDashboard.putNumber("past_gryo", gyro_angle);


    SmartDashboard.putNumber("final_vecx", final_vec.get(0, 0));
    SmartDashboard.putNumber("final_vecy", final_vec.get(1, 0));
    SmartDashboard.putNumber("final_vecz", final_vec.get(2, 0));
    
    var hub_coordinates = VecBuilder.fill( 8.255, 4.103, 2.64);

    var position =  hub_coordinates.minus(final_vec);

    // Print output
    SmartDashboard.putNumber("position_vecx", position.get(0, 0));
    SmartDashboard.putNumber("position_vecy", position.get(1, 0));
    SmartDashboard.putNumber("position_vecz", position.get(2, 0)); 

    if( (position.get(0,0) > 0 ) && (position.get(0,0) < fieldlength ) && (position.get(1,0) > 0) && (position.get(1,0) < fieldwidth) && (position.get(2,0) > - 0.9) && (position.get(2,0) <  0.9)){
 
      var o2vtranslation = (new Translation2d(position.get(0, 0),position.get(1, 0)).minus(getPastPose(elapsedtime).getEstimatedPosition().getTranslation()));
      //var o2vtransform = new Transform2d(o2vtranslation, new Rotation2d(0));
      SmartDashboard.putNumber("offset", o2vtranslation.getX()); 

      for (past_object past_object : past_positions) {
       past_object.estimate = addTranslation(past_object.estimate, o2vtranslation);
      }

      Pose2d final_position = addTranslation(SubsystemFactory.getInstance().getDrivetrain().getLocation(), o2vtranslation);
      

      SmartDashboard.putNumber("posx", final_position.getX());
      SmartDashboard.putNumber("posy", final_position.getY());
      // final_position = new Pose2d(position.get(0, 0), position.get(1,0), gyro.getRotation2d());
      if(!getPastPose(elapsedtime).getIsMoving()) {
        odometry.resetPosition(final_position, gyro.getRotation2d());
      }
      SmartDashboard.putNumber("Difference in LastVision",  final_position.getTranslation().minus(last_visionmeasurement.getTranslation()).getNorm());
      if(0.20 > final_position.getTranslation().minus(last_visionmeasurement.getTranslation()).getNorm()){
        laststabletime = Timer.getFPGATimestamp();
      }
      last_visionmeasurement = final_position;
      lastVisionTime = Timer.getFPGATimestamp();
      SmartDashboard.putNumber("Last Vision Time", lastVisionTime);

    }

  }


  public Pose2d addTranslation(Pose2d pose, Translation2d translation){
    return new Pose2d(pose.getTranslation().plus(translation), pose.getRotation());
  }


  public static double getlaststabletime() {
    return laststabletime;
  }

  public past_object getPastPose(double elapsedtime){
    for(int i = past_positions.size()-1; i >= 0; i--){
      var timeSinceMeasurement = Timer.getFPGATimestamp() - past_positions.get(i).getFPGATimestamp();
      if (timeSinceMeasurement> elapsedtime){
        SmartDashboard.putNumber("Index of Past Storage", i);
        SmartDashboard.putNumber("Time since measurement", timeSinceMeasurement);
        return past_positions.get(i);
      }
    } 
    return new past_object(new Pose2d(), 0.0, false);
  }

  public double getDistanceFromHub() {
    Translation2d botPosition = SubsystemFactory.getInstance().getDrivetrain().getSwerveDriveOdometry().getPoseMeters().getTranslation();
    Translation2d hubPosition = new Translation2d(8.25, 4.10);

    return botPosition.getDistance(hubPosition);
  }
  
  @Override
  public void periodic() { //called every 2 milliseconds
    SwerveDrivetrain drivetrain = SubsystemFactory.getInstance().getDrivetrain();
    
    if (past_positions.size()>100)
      past_positions.remove(0);
    past_positions.add(new past_object(drivetrain.getLocation(), Timer.getFPGATimestamp(), drivetrain.isMoving()));
   
    
  }

  @Override
  public void simulationPeriodic() {
    periodic();
  }


}
