package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.subsystems.Base.*;
import static frc.robot.Constants.*;

import java.util.Arrays;

public class Limelight extends SubsystemBase {
  private NetworkTable table;
  private String name = "limelight";
  
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
    table = NetworkTableInstance.getDefault().getTable(name);
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

    
    targetFound = table.getEntry("tv").getDouble(0);
    x = table.getEntry("tx").getDouble(0);
    y = table.getEntry("ty").getDouble(0);
    z = table.getEntry("tz").getDouble(0);
    area = table.getEntry("ta").getDouble(0);
    id = table.getEntry("tid").getDouble(0);
    skew = table.getEntry("ts").getDouble(0);
    botPose = table.getEntry("botpose").getDoubleArray(new double[6]);

    if (botPose.length != 0) {
      botPoseX = botPose[0];
      botPoseY = botPose[1];
    }
    
    
    String botPosString = Arrays.toString(botPose);
    SmartDashboard.putString("Botpose", botPosString);
    SmartDashboard.putNumber("Tid", skew);
    
    SmartDashboard.putNumber("BotposeX", botPoseX);
    SmartDashboard.putNumber("BotposeY", botPoseY);
  

    SmartDashboard.putNumber("pipeline", pipeline);
    // SmartDashboard.putData("stream deck", table.getEntry(""));

  }

  public void LEDOn() {
    // Eye Protection
    table.getEntry("ledMode").setNumber(3); // (turns limelight on)
  }

  public void LEDOff() {
    // Eye Protection
    table.getEntry("ledMode").setNumber(1); // (turns limelight off)
  }

  public void LEDBlink() {
    table.getEntry("ledMode").setNumber(2); // (blinks limelight)
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

  public double getYOffset() {
    return botPoseY;
  }

  public double getXOffset() {
    return botPoseX;
  }

  public double getAngle() {
    return x;
  }



  public double getTID() {
    return id;
  }
  public double getSkew() {
    return skew; 
  }

  // public double getYDistanceToPole() {
  // if (!getTargetFound()) return kDistanceWhenNoTarget;
  // double distanceToPole = getDistance();
  // // double angle = Rotation2d.getDe(base.getHeading());
  // double yDistance = distanceToPole;
  // }

  public double getDistance() {
    // default value to return when limelight does not see target
    if (!getTargetFound()) {
      return kDistanceWhenNoTarget;
    }
    double distance = Math.abs(KHeightDifference) / Math.tan(Math.toRadians(KLimelightAngle + Math.abs(y)));
    return distance + KDistanceOffset; // constant offset for this specific bot
  }

  public double getHorizontinalDistance() {
    if (!getTargetFound()) {
      return kHorizDistanceWhenNoTarget;
    }

    double distance = getDistance() / Math.tan(Math.toRadians(KLimelightAngle + x));
    return distance + KHorizDistanceOffset; // constant offset for this specific bot
  }
}
