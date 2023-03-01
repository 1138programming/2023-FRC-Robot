// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // Sensor Ports
    // BASE
    // Values will be changed for CANCoders (which will not use DIO)
  public static final int KFrontLeftMagEncoderID = 1;
  public static final int KFrontRightMagEncoderID = 2;
  public static final int KBackLeftMagEncoderID = 3;
  public static final int KBackRightMagEncoderID = 4;
  // DIO
    // Orientation
  public static final int KOrientationHallEffectSensor1ID = 1;
  public static final int KOrientationHallEffectSensor2ID = 2;
  public static final int KOrientationkDoorControlID = 3;
  public static final int KOrientationkBaseCheckerID = 4;
  public static final int KOrientationkTipCheckerID = 5;

  public static enum ORIENTATIONSTATE {
    CUBE,
    CONEBASE,
    CONETIP,
    NOTHING
  };

    // Scoring
  public static final int KScoringTopLimitSwitch = 6;
  public static final int KScoringBottomLimitSwitch = 7;
    // Endgame
  public static final int KEndgameLeftIR = 7;
  public static final int KEndgameRightIR = 8;
    // Intake
  public static final int KIntakeLimitId = 10;  //NavX port number: 0
  public static final int KIntakeEncoderID = 11; //NavX port number: 1
  
    // PWM
  public static final int KLinearServoFront = 0;
  public static final int KLinearServoBack = 1;

    // LED ports: (setup undetermined)
  public static final int KLEDPort = 2;
  public static final int KLEDBuffer = 20;
  public static enum KLEDSTATE {
    OFF,
    YELLOW,
    PURPLE
  };

  // Motor IDs
    // Base
  public static final int KFrontLeftAngleID = 2;
  public static final int KFrontLeftDriveID = 3;
  
  public static final int KFrontRightAngleID = 5;
  public static final int KFrontRightDriveID = 4;
  
  public static final int KBackLeftAngleID = 1;
  public static final int KBackLeftDriveID = 8;
  
  public static final int KBackRightAngleID = 7;
  public static final int KBackRightDriveID = 6;
    // Intake
  public static final int KSpaghettiIntakeId = 9;
  public static final int KFlexIntakeId = 11;
  public static final int KSwivelIntakeId = 10;
    // Storage
  public static final int KOrientationLeftMotorID = 12;
  public static final int KOrientationRightMotorID = 13;
  public static final int KOrientationMotorExtensionID = 14;
    // Scoring
  public static final int KClawMotor = 15;
  public static final int KWristMotor = 16;
  public static final int KLiftMotor = 17;
  public static final int KFlipperMotor = 18;
  

  // Math and other Constants
  // Swerve Modules
  public static final double KAngleP = 0.006;
  public static final double KAngleI = 0;
  public static final double KAngleD = 0;
  
  public static final double KDriveP = 0.2;
  public static final double KDriveI = 0.75;
  public static final double KDriveD = 0.005;
  
    // used for math in Constants and SwerveModules to set up encoder units
  public static final double KDegPerRotation = 360;
  public static final double KNeoMaxRPM = 5700;
  
    // Sets up drive encoders to use meters as the unit
  private static final double KDriveMotorGearRatio = 1/6.55;
  private static final double KWheelDiameterMeters = 0.1016;
  public static final double KDriveMotorRotToMeter = KDriveMotorGearRatio * KWheelDiameterMeters * Math.PI;
  
  public static final double KDriveMotorRPMToMetersPerSec = KDriveMotorRotToMeter / 60;
  
  public static final double KAngleMotorRotToDeg = 35;
  public static final double KPhysicalMaxDriveSpeedMPS = KNeoMaxRPM * KDriveMotorRPMToMetersPerSec;
  public static final double KMaxAngularSpeed = Math.PI; // MAY NEED CHANGING
  
  // not used, may be useful to know/keep
  private static final double KAngleMotorShaftToWheelRatio = 1 / 10.2857; 
  
  
    // Low and high percent: sets max speed of drivetrain for driver
  public static final double KBaseDriveLowPercent = 0.3;
  public static final double KBaseDriveHighPercent = 0.7;
  public static final double KBaseDriveMaxPercent = 1;
  
    // Offsets for absolute encoders, used to set up angle encoders
  // public static final double KFrontLeftOffset = 0.159;
  public static final double KFrontLeftOffset = -147.22;
  public static final double KFrontRightOffset = -36.56;
  public static final double KBackLeftOffset = -340.14;
  public static final double KBackRightOffset = -47.37;
  
  
    // Describes the locations of the swerve modules relative to the center of the robot
    // Important for kinematics
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

    // Setting up which motors are reversed
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
  
  public static final double KIntakeSwivelTopPos = 11;
  public static final double KIntakeSwivelBottumPos = 12;

  public static final double KIntakeConeSpaghettitSpeed = 0.35;

  
  public static final double KIntakeCubeSpaghettitSpeed = 0.35;

  
  public static final double KIntakeSwiveTopOffset = 2; 
  public static final double KIntakeSwiveBottumOffset = 2;  
  
  
  // Intake
    // Swivel PID
  public static final double KIntakeP = 0;
  public static final double KIntakeI = 0;
  public static final double KIntakeD = 0;
  
  // Endgame
  public static final double KEndgameServoReadyPos = 1;
  public static final double KEndgameServoNeutralPos = 0.74;
  public static final double KEndgameServoDeployPos = 0.41;
  
  // Scoring
  public static final int KScoringEncoder1ID = 6; //name can be changed later
  public static final int KScoringEncoder2ID = 7; //name can be changed later
    // Flipper PID
  public static final double KFlipperP = 0;
  public static final double KFlipperI = 0;
  public static final double KFlipperD = 0;
  public static final double KScoringFlipPos = 0;
    // lift PID
  public static final double KLiftP = 0; //TBD
  public static final double KLiftI = 0; //TBD
  public static final double KLiftD = 0; //TBD
  public static final double KLiftRotToFoot = 0; //TBD
  
  // Orientation
  public static final double KMotorExtensionSpeed = 0.5;
  public static final double KMotorExtensionTime = 0.5;
  public static final double KCubeLeftandRightMotorSpeeds = 18;
  public static final double KConeLeftandRightMotorSpeeds = 18;
  
  // Scoring
  public static final double KClawMotorSpeed = 0; //TBD
  public static final double KAngleMotorSpeed = 0; //TBD
  public static final double KExtensionMotorSpeed = 0; //TBD
  
  //Limelight
  public static final double KLimelightHeight = 19.5; // inches
  public static final double KMidPoleHeight = 25; // inches
  // public static final double KHighPoleHeight = 0; // inches
  public static final double KHeightDifference = KMidPoleHeight - KLimelightHeight; // inches
  public static final double KLimelightAngle = -5;
  public static final double KLimelightRange = 29.8;
  public static final double kDistanceWhenNoTarget = 0;
  public static final double kHorizDistanceWhenNoTarget = 0;
  public static final double kDesiredYOffset = 1;
  public static final double kDesiredXOffset = 1;
  public static final double kLimelightXOffsetDeadzone = 0.05;
  public static final double KDistanceOffset = 0;
  public static final double KHorizDistanceOffset = 0;
  public static final double KGoalWidth = 15;

  public static final double KDistanceMoveOffset = 1;

  public static final double KOffsetFromAprilTag = 1;

  public static final double[] KXCoordinateOfTag = { 
    7.24310,
    7.24310,
    7.24310,
    7.90832,
    -7.90832,
    -7.24310,
    -7.24310,
    -7.24310,
  };

  public static final double[] KYCoordinateOfTag = { 
    -2.93659,
    -1.26019,
    0.41621,
    2.74161,
    2.74161,
    0.41621,
    -1.26019,
    -2.93659
  };

  public static final double KLimelightRotateP = 0.01;
  public static final double KLimelightRotateI = 0;
  public static final double KLimelightRotateD = 0;
  public static final double KLimelightMoveDeadzone = 0.01;

  
  public static final double KTagLimelightMoveP = 0.08;
  public static final double KTagLimelightMoveI = 0;
  public static final double KTagLimelightMoveD = 0;

  public static final double KTapeLimelightMoveP = 0.005;
  public static final double KTapeLimelightMoveI = 0;
  public static final double KTapeLimelightMoveD = 0;


}
