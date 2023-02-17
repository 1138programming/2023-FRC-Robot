package frc.robot.subsystems;

import static frc.robot.Constants.*;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;

import java.util.ArrayList;

import com.fasterxml.jackson.databind.node.POJONode;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Base extends SubsystemBase {
  private SwerveModule frontLeftModule;
  private SwerveModule frontRightModule;
  private SwerveModule backLeftModule;
  private SwerveModule backRightModule;

  private AHRS gyro;

  private SwerveDriveKinematics kinematics;
  private SwerveDriveOdometry odometry;
  private Pose2d pose;

  
  private double driveSpeedFactor;

  // logging stuff: not currently being used
  private boolean enableLogging;
  private long startTime;
  private ArrayList<String> odometryData;
  private static final String logFolder = "/media/sda2/";
  private static final String logTitle = "'OdometryLog'_yy-MM-dd_HH-mm-ss'.csv'";
  
  public Base() {
    frontLeftModule = new SwerveModule(
      KFrontLeftAngleID,
      KFrontLeftDriveID,
      KFrontLeftMagEncoderID,
      KFrontLeftOffset,
      KFrontLeftDriveReversed,
      KFrontLeftAngleReversed
    );
    frontRightModule = new SwerveModule(
      KFrontRightAngleID,
      KFrontRightDriveID,
      KFrontRightMagEncoderID, 
      KFrontRightOffset,
      KFrontRightDriveReversed,
      KFrontRightAngleReversed
    );
    backLeftModule = new SwerveModule(
      KBackLeftAngleID,
      KBackLeftDriveID,
      KBackLeftMagEncoderID, 
      KBackLeftOffset,
      KBackLeftDriveReversed,
      KBackLeftAngleReversed
    );
    backRightModule = new SwerveModule(
      KBackRightAngleID,
      KBackRightDriveID,
      KBackRightMagEncoderID, 
      KBackRightOffset,
      KBackRightDriveReversed,
      KBackRightAngleReversed
    );

    gyro = new AHRS(SPI.Port.kMXP);
    gyro.reset();

    kinematics = new SwerveDriveKinematics(
      KFrontLeftLocation, KFrontRightLocation,
      KBackLeftLocation, KBackRightLocation
    );
    odometry = new SwerveDriveOdometry(kinematics, getHeading(), getPositions());
    driveSpeedFactor = KBaseDriveLowPercent;

    enableLogging = false;
    startTime = RobotController.getFPGATime();
    odometryData = new ArrayList<String>();
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, double maxDriveSpeedMPS) {
    xSpeed *= maxDriveSpeedMPS;
    ySpeed *= maxDriveSpeedMPS;
    rot *= KMaxAngularSpeed;
    
    //feeding parameter speeds into toSwerveModuleStates to get an array of SwerveModuleState objects
    SwerveModuleState[] states =
      kinematics.toSwerveModuleStates(
        fieldRelative
          ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getHeading())
          : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(states, KPhysicalMaxDriveSpeedMPS);
    SmartDashboard.putNumber("speedFactor", driveSpeedFactor);

    //setting module states, aka moving the motors
    frontLeftModule.setDesiredState(states[0]);
    // frontRightModule.setDesiredState(states[1]);
    // backLeftModule.setDesiredState(states[2]);
    // backRightModule.setDesiredState(states[3]);
}

  // recalibrates gyro offset
  public void resetGyro() {
    gyro.reset(); 
    gyro.setAngleAdjustment(0);
  }

  public void resetAllRelEncoders() {
    frontLeftModule.resetRelEncoders();
    frontRightModule.resetRelEncoders();
    backLeftModule.resetRelEncoders();
    backRightModule.resetRelEncoders();
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    positions[0] = frontLeftModule.getPosition();
    positions[1] = frontRightModule.getPosition();
    positions[2] = backLeftModule.getPosition();
    positions[3] = backRightModule.getPosition();

    return positions;
  }
  
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(getHeadingDeg());
  }

  public double getHeadingDeg() {
    return -gyro.getAngle();
  }

  public void resetOdometry() {
    resetAllRelEncoders();
    pose = new Pose2d();
    
    odometry.resetPosition(getHeading(), getPositions(), pose);
  }

  public void setLoggingEnabled(boolean enableLogging) {
    this.enableLogging = enableLogging;
  }
  public boolean getLoggingEnabled() {
    return enableLogging;
  }

  private void genOdometryData() {
    long time =  RobotController.getFPGATime() - startTime;
    String s = ("" + (double) time / 1000000);
    s += "," + pose.getX() + "," + pose.getY() + "," + pose.getRotation().getDegrees();

    odometryData.add(s);
  }

  public void writeOdometryData() {
    try {
      FileWriter writer = new FileWriter("OdometryLog.txt");
      for (int i = 0; i < 1000; i++) {
        writer.write(odometryData.get(i));
      }
      writer.close();
    } catch (IOException e) {
      System.out.println("An error occurred.");
      e.printStackTrace();
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Gyro", getHeadingDeg());

    SmartDashboard.putNumber("Front left module", frontLeftModule.getAngleDeg());
    SmartDashboard.putNumber("Front right module", frontRightModule.getAngleDeg());
    SmartDashboard.putNumber("Back left module", backLeftModule.getAngleDeg());
    SmartDashboard.putNumber("Back right module", backRightModule.getAngleDeg());

    SmartDashboard.putNumber("front left mag", frontLeftModule.getMagDeg());
    SmartDashboard.putNumber("front right mag", frontRightModule.getMagDeg());
    SmartDashboard.putNumber("back left mag", backLeftModule.getMagDeg());
    SmartDashboard.putNumber("back right mag", backRightModule.getMagDeg());

    SmartDashboard.putString("odometry pose", odometry.getPoseMeters().toString());

    SmartDashboard.putBoolean("isCalibrating", gyro.isCalibrating());

    odometry.update(getHeading(), getPositions());
    pose = odometry.getPoseMeters();

    if (enableLogging && odometryData.size() < 1000) {
      genOdometryData();
    }
  }

  public double getDriveSpeedFactor()
  {
    return driveSpeedFactor;
  }
  public void setDriveSpeedFactor(double set)
  {
    driveSpeedFactor = set;
  }
}
  
  