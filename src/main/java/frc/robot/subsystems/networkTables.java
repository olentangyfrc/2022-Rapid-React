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
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;





public class networkTables extends SubsystemBase {

  private SwerveDriveOdometry odometry;
  private ArrayList<past_object> past_positions = new ArrayList<past_object>(100);

  private int fieldlength = 16;
  private int fieldwidth = 8;

  NetworkTableInstance inst;
  Gyro gyro;
  NetworkTableEntry Time, XEntry, YEntry, ZEntry;

  public class past_object{
    private Pose2d estimate;
    private double timestamp;
    public past_object(Pose2d estimate, double timestamp){
      this.estimate = estimate;
      this.timestamp = timestamp;
    }
    public Pose2d getEstimatedPosition(){
      return estimate;
    }
    public double getFPGATimestamp(){
      return timestamp;
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

    Time.addListener(event -> {onVisionUpdate(event);}, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
   
  }

  
  public void onVisionUpdate(EntryNotification event ){
    
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
    // Build our rotation matrix
    var pitch = -28 * Math.PI / 180;
    var c = Math.cos(pitch);
    var s = Math.sin(pitch);
    var camera_to_bot = Matrix.mat(Nat.N3(), Nat.N3()).fill(
        c, 0, s,
        0, 1, 0,
      -s, 0, c);

    // Rotate
    var corrected_bot_oriented = camera_to_bot.times(corrected_vec);


    var trans2_vec = VecBuilder.fill(0.0, -0.215, 0.813); //Change this where we know the displacement of the camera to the center of the robot

    var out_vec = trans2_vec.plus(corrected_bot_oriented);
    double gyro_angle = 0; //add the heading by multipling by gyro angle
    
    
   
    gyro_angle = past_positions.get(getPastPose(elapsedtime)).getEstimatedPosition().getRotation().getRadians();
    c = Math.cos(gyro_angle);
    s = Math.sin(gyro_angle);
    var bot_to_field = Matrix.mat(Nat.N3(), Nat.N3()).fill(
        c, -s, 0,
        s, c, 0,
        0, 0, 1);
    var final_vec = bot_to_field.times(out_vec);
    
    var hub_coordinates = VecBuilder.fill( 8.255, 4.103, 2.64);

    var position =  hub_coordinates.minus(final_vec);

    // Print output

    SmartDashboard.putNumber("position_vecx", position.get(0, 0));
    SmartDashboard.putNumber("position_vecy", position.get(1, 0));
    SmartDashboard.putNumber("position_vecz", position.get(2, 0)); 

    if( (position.get(0,0) > 0 ) && (position.get(0,0) < fieldlength ) && (position.get(1,0) > 0) && (position.get(1,0) < fieldwidth) && (position.get(2,0) > - 0.9) && (position.get(2,0) <  0.9)){

      Transform2d visiontoodermetry = new Transform2d();       
      visiontoodermetry = (new Pose2d(position.get(0, 0),position.get(1, 0), gyro.getRotation2d()).minus(past_positions.get(getPastPose(elapsedtime)).getEstimatedPosition()));

      SmartDashboard.putNumber("offset", visiontoodermetry.getX()); 

      for (past_object past_object : past_positions) {
        past_object.estimate = past_object.estimate.plus(visiontoodermetry.times(0.6));
      }

      Pose2d final_position = odometry.getPoseMeters().plus(visiontoodermetry.times(0.6));


      SmartDashboard.putNumber("x", final_position.getX());
      SmartDashboard.putNumber("y", final_position.getY());
      if (final_position.getY() == 0){
       odometry.resetPosition(final_position, gyro.getRotation2d());
      }  
    }
    

  }

  public int getPastPose(double elapsedtime){
    for(int i = past_positions.size()-1; i >= 0; i--){
      if ((Timer.getFPGATimestamp() - past_positions.get(i).getFPGATimestamp()) > elapsedtime){
        return i;
      }
    } 
    return 0;
  }

  public double getDistanceFromHub() {
    Translation2d botPosition = SubsystemFactory.getInstance().getDrivetrain().getSwerveDriveOdometry().getPoseMeters().getTranslation();
    Translation2d hubPosition = new Translation2d(8.23, 4.115);

    return botPosition.getDistance(hubPosition);
  }
  
  @Override
  public void periodic() { //called every 2 milliseconds
    
    if (past_positions.size()>100)
      past_positions.remove(0);
    past_positions.add(new past_object(odometry.getPoseMeters(), Timer.getFPGATimestamp()));
   
    
  }

  @Override
  public void simulationPeriodic() {
    periodic();
  }


}
