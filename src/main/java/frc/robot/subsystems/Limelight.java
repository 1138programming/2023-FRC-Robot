package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.Base;

import frc.robot.subsystems.Base;
import static frc.robot.Constants.*;

import java.util.Arrays;

public class Limelight extends SubsystemBase {
  private NetworkTable aprilTagsTable;
  private NetworkTable tapeTable;
  private NetworkTable defaultTable;
  private String aprilTagsPipeline = "AprilTags";
  private String tapePipeline = "Tape";
  private String defaultPipeline = "limelight";

  
  
  private double targetFound;
  private double x;
  private double y;
  private double z;
  private double id;
  private double skew;
  private double area;
  private double[] botPose;
  private double pipeline;
  private double botPoseX;
  private double botPoseY;



  public Limelight() {
    aprilTagsTable = NetworkTableInstance.getDefault().getTable(defaultPipeline);
    tapeTable = NetworkTableInstance.getDefault().getTable(defaultPipeline);
    targetFound = 0;
    x = 0;
    y = 0;
    z = 0; 
 
    id = 0;
    area = 0;
    skew = 0;
    botPoseX = 1;
    botPoseY = 1;
    
   
    

    botPose = new double[6];
  }

  @Override
  public void periodic() {
    // getting limelight networktable values
    if (pipeline == 0) {
      targetFound = aprilTagsTable.getEntry("tv").getDouble(0);
      x = aprilTagsTable.getEntry("tx").getDouble(0);
      y = aprilTagsTable.getEntry("ty").getDouble(0);
      z = aprilTagsTable.getEntry("tz").getDouble(0);
      area = aprilTagsTable.getEntry("ta").getDouble(0);
      id = aprilTagsTable.getEntry("tid").getDouble(0);
      botPose = aprilTagsTable.getEntry("botpose").getDoubleArray(new double[6]);

     

    }
    else if (pipeline == 1) {
      targetFound = tapeTable.getEntry("tv").getDouble(0);
      x = tapeTable.getEntry("tx").getDouble(0);
      y = tapeTable.getEntry("ty").getDouble(0);
      z = tapeTable.getEntry("tz").getDouble(0);
      area = tapeTable.getEntry("ta").getDouble(0);  
    }
    
    if (botPose.length != 0) {
      botPoseX = botPose[0];
      botPoseY = botPose[1];
    }
  }

  public void LEDOn() {
    // Eye Protection
    getTable().getEntry("ledMode").setNumber(3); // (turns limelight on)
  }

  public void LEDOff() {
    // Eye Protection
    getTable().getEntry("ledMode").setNumber(1); // (turns limelight off)
  }

  public void LEDBlink() {
    getTable().getEntry("ledMode").setNumber(2); // (blinks limelight)
  }

  

  public boolean getTargetFound() {
    if (targetFound == 0) {
      return false;
    } else if (targetFound == 1) {
      return true;
    } else {
      return false;
    }
  }

  public double getTagYOffset() {
    return botPoseY;
  }

  public double getTagXOffset() {
    return botPoseX;
  }

  public double getXAngle() {
    return x;
  }
  public double getYAngle() {
    return x;
  }

  public double getArea() {
    return area;
  }

  public NetworkTable getTable() {
    return defaultTable;
    // if (pipeline == 0) {
    //   return aprilTagsTable;
    // }
    // return tapeTable;
  }

  public String getTableString() {
    if (pipeline == 0) {
      return "AprilTags";
    }
    return "Tape";
  }

  public double getPipeline() {
    return pipeline;
  }
  public void setPipeline(int pipeline) {
    this.pipeline = pipeline;
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline); //(turns limelight on)
  }


/**
 * Get ID of nearest AprilTag
 */
  public double getTID() {
    return id;
  }
  public double getSkew() {
    return skew; 
  }

  public double getDistance() {
    // default value to return when limelight does not see target
    if (!getTargetFound()) {
      return kDistanceWhenNoTarget;
    }
    double distance = Math.abs(KHeightDifference) / Math.tan(Math.toRadians(KLimelightAngle + Math.abs(y)));
    return distance + KDistanceOffset; // constant offset for this specific bot
  }
  

  public double getHorizontinalDistance(Base base) {
    if (!getTargetFound()) {
      return kHorizDistanceWhenNoTarget;
    }
    else
    {
      double distance = Math.abs(getDistance()) / Math.tan(Math.toRadians(90-(base.getHeadingDeg() % 90) + x));
      return distance + KHorizDistanceOffset; // constant offset for this specific bot
    }
    
   
  }
}
