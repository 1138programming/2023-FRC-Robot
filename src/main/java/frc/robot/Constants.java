// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Enumeration;

import com.fasterxml.jackson.databind.introspect.AnnotationCollector.OneAnnotation;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  //Swerve Module
  public static final double KAngleP = 0.006;
  public static final double KAngleI = 0;
  public static final double KAngleD = 0;
  
  public static final double KDriveP = 0.2;
  public static final double KDriveI = 0.75;
  public static final double KDriveD = 0.005;
  
  public static final double KDegPerRotation = 360;
  
  private static final double KDriveMotorGearRatio = 1/6.55;
  private static final double KWheelDiameterMeters = 0.1016;
  public static final double KDriveMotorRotToMeter = KDriveMotorGearRatio * KWheelDiameterMeters * Math.PI;

  public static final double KDriveMotorRPMToMetersPerSec = KDriveMotorRotToMeter / 60;

  private static final double KAngleMotorShaftToWheelRatio = 1 / 10.2857;
  public static final double KAngleMotorRotToDeg = 35;
  public static final double KNeoMaxRPM = 5700;
  public static final double KPhysicalMaxDriveSpeedMPS = KNeoMaxRPM * KDriveMotorRPMToMetersPerSec;
  public static final double KMaxAngularSpeed = Math.PI;
  
  
  //Base
  public static final int KFrontLeftAngleID = 5;
  public static final int KFrontLeftDriveID = 4;
  
  public static final int KFrontRightAngleID = 7;
  public static final int KFrontRightDriveID = 6;
  
  public static final int KBackLeftAngleID = 2;
  public static final int KBackLeftDriveID = 3;
  
  public static final int KBackRightAngleID = 1;
  public static final int KBackRightDriveID = 8;
  
  public static final double KBaseDriveLowPercent = 0.5;
  public static final double KBaseDriveHighPercent = 0.7;
  
  public static final double KFrontLeftOffset = 0.159;
  public static final double KFrontRightOffset = 0.5047;
  public static final double KBackLeftOffset = 0.2044;
  public static final double KBackRightOffset = 0.192;
  
  public static final int KFrontLeftMagEncoderID = 7;
  public static final int KFrontRightMagEncoderID = 5;
  public static final int KBackLeftMagEncoderID = 2;
  public static final int KBackRightMagEncoderID = 0;
  
  public static final double KWheelDistanceFromCenter = 0.29845;
  
  public static final Translation2d KFrontLeftLocation = new Translation2d(
    KWheelDistanceFromCenter, KWheelDistanceFromCenter
  );
  public static final Translation2d KFrontRightLocation = new Translation2d(
    KWheelDistanceFromCenter, -KWheelDistanceFromCenter
  );
  public static final Translation2d KBackLeftLocation = new Translation2d(
    -KWheelDistanceFromCenter, KWheelDistanceFromCenter
  );
  public static final Translation2d KBackRightLocation = new Translation2d(
    -KWheelDistanceFromCenter, -KWheelDistanceFromCenter
  );

  public static final boolean KFrontLeftDriveReversed = false;
  public static final boolean KFrontRightDriveReversed = false;
  public static final boolean KBackLeftDriveReversed = false;
  public static final boolean KBackRightDriveReversed = false;

  public static final boolean KFrontLeftAngleReversed = true;
  public static final boolean KFrontRightAngleReversed = true;
  public static final boolean KBackLeftAngleReversed = true;
  public static final boolean KBackRightAngleReversed = true;

  public static final boolean KFrontLeftDriveEncoderReversed = false;
  public static final boolean KFrontRightDriveEncoderReversed = false;
  public static final boolean KBackLeftDriveEncoderReversed = false;
  public static final boolean KBackRightDriveEncoderReversed = false; 

  //intake
  public static final int KSpaghettiIntakeId = 12;
  public static final int KLeftIntakeId = 0;
  public static final int KRightIntakeId = 13;

  public static final double KIntakeSpaghettitSpeed = 0.35;
  public static final double KIntakeRollerSpeed = 0.35;

  public static final double KIntakeSwivelTopPos = 1;
  public static final double KIntakeSwivelBottumPos = 1;

  public static final int KIntakeLimitId = 1; 


  //Pid
  public static final double KIntakeP = 0;
  public static final double KIntakeI = 0;
  public static final double KIntakeD = 0;

  // Endgame
  public static final int KLimitSwitch = 0;
  public static final int KLinearServoTop = 0;

  public static final int KLinearServoBottom = 3;

  public static final double KEndgameServoPos = 1;
  // Scoring
  public static final int KClawMotor = 15;
  public static final int KWristMotor = 16;
  public static final int KLiftMotor = 17;
  public static final int KFlipperMotor = 18;

  public static final boolean KExtensionMotor1Reversed = true;
  public static final int KClawSolenoidForwardChannel = 1;
  public static final int KClawSolenoidReverseChannel = 2;


  public static final double KClawMotorSpeed = 0; //TBD
  public static final double KAngleMotorSpeed = 0; //TBD
  public static final double KExtensionMotorSpeed = 0; //TBD

  public static final int KOrientationSensor1ID = 0;
  public static final int KOrientationSensor2ID = 0;
  public static final int KOrientationSensor3ID = 0;
  public static final int KOrientationLeftMotorID = 12;
  public static final int KOrientationRightMotorID = 13;
  public static final int KOrientationMotorExtensionID = 14;
  public static final double KLeftandRightMotorSpeeds = 18;
  public static final double KMotorExtensionSpeed = 20;
          
  //LED ports
  public static final int KLEDPort = 2;
  public static final int KLEDBuffer = 20;
  public static enum KLEDSTATE {
    OFF,
    YELLOW,
    PURPLE
  };

 
}
