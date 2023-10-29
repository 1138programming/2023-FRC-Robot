// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
  
  // Sensing - CANBUS -------------------------------------------------------------
  
  // BASE - Values will be changed for CANCoders (which will not use DIO)
  public static final int KFrontLeftMagEncoderID = 1;
  public static final int KFrontRightMagEncoderID = 2;
  public static final int KBackLeftMagEncoderID = 3;
  public static final int KBackRightMagEncoderID = 4;
  public static final int KIntakeCanCoder = 5;
  public static final int KFlipperCanCoder = 6;
  
  // End of Sensing - CANBUS **************************************************
  
  // Scoring - 2 in total
  
  public static final int KScoringTopLimitSwitch = 6;
  public static final int KScoringBottomLimitSwitch = 9;
  public static final int KLiftEncoderA = 2; // NavX port number: 0 
  public static final int KLiftEncoderB = 3; // NavX port number: 0 
  
  // Scoring CANcoders 
  public static final int KClawCanCoder = 25;
  
  
    public static final int KIntakeTopLimitId = 4; 
  // Intake - 3 in total
  public static final int KIntakeBottomLimitId = 8; 
  public static final int KIntakeEncoderID = 9;
  
  // End of Sensing - DIO *****************************************************
  
  // Servos - PWM ----------------------------------------------------------------
  
  public static final int KLEDPort = 5;
  
  // End Servo Section ********************************************************
  
  // Motor IDs by Subsystem ------------------------------------------------------
  
  // Base
  public static final int KFrontLeftAngleID = 9;  	// SparkMax + NEO
  public static final int KFrontLeftDriveID = 8;  	// SparkMax + NEO
  
  public static final int KFrontRightAngleID = 2;  	// SparkMax + NEO
  public static final int KFrontRightDriveID = 1;  	// SparkMax + NEO
  
  public static final int KBackLeftAngleID = 11;  	  // SparkMax + NEO
  public static final int KBackLeftDriveID = 10;  	  // SparkMax + NEO
  
  public static final int KBackRightAngleID = 19;  	// SparkMax + NEO
  public static final int KBackRightDriveID = 18;  	// SparkMax + NEO
  
  // Intake
  public static final int KSpaghettiIntakeId = 7;   // Talon + 775
  public static final int KSwivelIntakeId = 6;     // Talon + 775
  
  
  // Scoring
  public static final int KFlipperRollerMotor = 15; //SparkMax + Neo
  public static final int KFlipperSwivelMotor = 16;	// Talon + 775
  public static final int KLiftMotor = 17;	
  
  // End of Motor Section *****************************************************
  
  // CURRENT LIMITS
  public static final int KDriveMotorCurrentLimit = 40;
  public static final int KAngleMotorCurrentLimit = 30;
  // public static final int KLiftMotorCurrentLimit = 30;
  // public static final int KLiftMotorCurrentLimit = 30;

  
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
  public static final double KMaxAngularSpeed = 3.5; 

  public static final double KTimedDriveSpeed = 0.3;
  
  // not used, may be useful to know/keep
  private static final double KAngleMotorShaftToWheelRatio = 1 / 10.2857;
  
    // Low and high percent: sets max speed of drivetrain for driver
  public static final double KBaseDriveLowPercent = 0.25;
  public static final double KBaseDriveMidPercent = 0.5;
  public static final double KBaseDriveMaxPercent = 1;

  public static final double KBaseRotLowPercent = 0.75;
  public static final double KBaseRotMidPercent = 1;
  public static final double KBaseRotMaxPercent = 1.5;
  
  // public static final int KDriveLowSpeed = 1;
  // public static final int KDriveMidSpeed = 2;
  // public static final int KDriveHighSpeed = 3;
  
    // Offsets for absolute encoders, used to set up angle encoders
    public static final double KFrontLeftOffset = -257.90;
    public static final double KFrontRightOffset = -37.00;
    public static final double KBackLeftOffset = -316.07;
    public static final double KBackRightOffset = -255.23;  
  
    // Describes the locations of the swerve modules relative to the center of the robot
  // Important for kinematics
  public static final double KWheelDistanceFromCenter = 0.257175;
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

  public static final double KGyroOffset = 180;
  
  
  
  public static final double KIntakeOffset = 150;
  public static final double KIntakeSwivelOffset = 1;
  public static final double KIntakeSwivelTopPos = 29;
  public static final double KIntakeSwivelShootPos = 59;
  public static final double KSwivelBottomPosition = 137;

  //Speeds and stuff
  public static final double KIntakeSpaghettiSpeed = 0.8;
  public static final double KIntakeSpaghettiShootSpeed = -0.85;
  
  public static final double KIntakeSwivelSpeed = 0.45;
  public static final double KIntakeSwivelMaxPassiveSpeed = 0.25;
  
  // Intake
  // Swivel Encoder
  // Swivel PID
  
  // Gray bot values
  // public static final double KIntakeP = 0.003;
  // public static final double KIntakeI = 0.0001;
  // public static final double KIntakeD = 0;
  
  // Blue bot values: TBD
  public static final double KIntakeP = 0.0065; // TBD
  public static final double KIntakeI = 0.001; // TBD 
  public static final double KIntakeD = 0; // TBD
  
  // public static final double KIntakeSwivelCanCoderRatio = 1;
  public static final double KIntakeSwivelCanCoderRatio = 14/34;

  // Timing for lift
  public static final double KIntakeThenLiftTime = 0.5;
  
  
  
  // Scoring
  public static final int KScoringEncoder1ID = 6; //name can be changed later
  public static final int KScoringEncoder2ID = 7; //name can be changed later
  
  // Flipper PID
  public static final double KFlipperP = 0.03; // TBD
  public static final double KFlipperI = 0; // TBD
  public static final double KFlipperD = 0; // TBD
  public static final double KScoringFlipPos = 0; //TBD

  public static final double KFlipperCanCoderRatio = 0.4375;
  
  // lift PID
  public static final double KLiftP = 0.0005; //TBD
  public static final double KLiftI = 0; //TBD
  public static final double KLiftD = 0.0000005; //TBD
  
  public static final double KInnerLiftP = 0; //TBD
  public static final double KInnerLiftI = 0; //TBD
  public static final double KInnerLiftD = 0; //TBD
  public static final double KLiftRotToFoot = 0; //TBD
  
  // Orientation
  public static final boolean KCubeMode = false;
  public static final boolean KConeMode = true;
  
  // Scoring
  //public static final double KClawMotorSpeed = 0.5; //TBD
  public static final double KClawMotorSpeed = 0.6;
  public static final double KClawSwivelMotorSpeed = 0.15;
  public static final double KAngleMotorSpeed = 0; //TBD
  public static final double KElevatorSpeed = 0.5; //TBD
  
  public static final double KLiftReadyPos = 0; //TBD
  public static final double KLiftLowPos = 0; //TBD
  public static final double KLiftMediumPos = 4284.75;
  public static final double KLiftHighPos = 8836.25; 
  public static final double KLiftShelfPos = 6988; //TBD
  
  public static final double KClawSwivelKP = 0.00001;
  public static final double KClawSwivelKI =0;
  public static final double KClawSwivelKd =0;
  
  // public static final double KFlipperInPos = 0;
  // public static final double KFlipperStartPos = 0;
  // public static final double KFlipperOutPos = 43;

  public static final double KLiftDeadzone = 2;
  public static final double KInnerLiftDeadzone = 4;
  

  // public static final double KInnerLiftHighPos= 0;
  // public static final double KInnerLiftLowPos= 0;
  
  public static final boolean KWristFlip = true; 
  
  public static final double KWristFlipPos = 0.86;
  public static final double KWristNoFlipPos = 0;
  public static final double KWristCubePos = 0.35;
  
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
  
  public static final int KAprilTagPipeline = 0;
  public static final int KReflectiveTapePipeline = 1;
  
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
  
  // Additional LED info:
  public static final int KLEDBuffer = 30;
  public static enum KLEDSTATE {
    OFF,
    YELLOW,
    PURPLE
  };
  public static final double KPPMaxVelocity = 4;
  public static final double KPPMaxAcceleration = 3;
  
  public static final double KXControllerP = 1;
  public static final double KXControllerI = 0;
  public static final double KXControllerD = 0;
  
  public static final double KYControllerP = 1;
  public static final double KYControllerI = 0;
  public static final double KYControllerD = 0;
  
  public static final double KRotControllerP = 1;
  public static final double KRotControllerI = 0;
  public static final double KRotControllerD = 0;
  
  public static final double KRotMaxVelocity = 6.28;
  public static final double KRotMaxAcceleration = 3.14;
  
  // Auto balance PID
  public static final double KBalanceP = 0.008;
  // public static final double KBalanceP = 0.0065;
  public static final double KBalanceI = 0;
  public static final double KBalanceD = 0.001;

  // Poses for DriveToPose autons
  public static final Pose2d KOpenSideCubePickupBlue = new Pose2d(4.3, -0.45, new Rotation2d());
  public static final Pose2d KOpenSideBackToCommunityBlue = new Pose2d(0.5, -0.2, Rotation2d.fromDegrees(145));
  public static final Pose2d KOpenSideShootPositionBlue = new Pose2d(0.27, -0.2, Rotation2d.fromDegrees(180));

  public static final Pose2d KOpenSideCubePickupRed = new Pose2d(4.3, 0.45, new Rotation2d());
  public static final Pose2d KOpenSideBackToCommunityRed = new Pose2d(0.5, 0.2, Rotation2d.fromDegrees(145));
  public static final Pose2d KOpenSideShootPositionRed = new Pose2d(0.27, 0.2, Rotation2d.fromDegrees(180));

  public static final Pose2d KCableSideCubePickupBlue = new Pose2d(4.3, 0.45, new Rotation2d());
  public static final Pose2d KCableSideCrossCableBlue = new Pose2d(2.85, 0.28, new Rotation2d());
  public static final Pose2d KCableSideCubePickupAfterCrossBlue = new Pose2d(4.5, 0.45, new Rotation2d());
  public static final Pose2d KCableSideBackToCommunityBlue = new Pose2d(0.3, 0, new Rotation2d());
  public static final Pose2d KCableSideShootPositionBlue = new Pose2d(0.3, 0, Rotation2d.fromDegrees(180));
  
  
  public static PathConstraints KPathPLannerConstraints = new PathConstraints(KPPMaxVelocity, KPPMaxAcceleration);

  // Pathplanner trajectories
  //left
  
  public static final PathPlannerTrajectory KLeftSideCubeToBalance = PathPlanner.loadPath("LeftSideCubeToBalance", KPathPLannerConstraints);
  public static final PathPlannerTrajectory KLeftSideCubeToStation = PathPlanner.loadPath("LeftSideCubeToStation", KPathPLannerConstraints);
  public static final PathPlannerTrajectory KPickUpLeftSide = PathPlanner.loadPath("PickUpLeftSide", new PathConstraints(KPPMaxVelocity, KPPMaxAcceleration));
  public static final PathPlannerTrajectory KLeftSidePickup2 = PathPlanner.loadPath("LeftSidePickup2", KPathPLannerConstraints);
  
  public static final PathPlannerTrajectory KLeftSidePickupToBalance = PathPlanner.loadPath("LeftSidePickupToBalance", new PathConstraints(2, 2));
  public static final PathPlannerTrajectory KLeftSidePickup2ToBalance = PathPlanner.loadPath("LeftSidePickup2ToBalance", new PathConstraints(2, 2));
  //Right
  
  public static final PathPlannerTrajectory KRightSideLeaveCommunity = PathPlanner.loadPath("LeftSideLeaveCommunity", new PathConstraints(KPPMaxVelocity, KPPMaxAcceleration));
  public static final PathPlannerTrajectory KRightSideLeaveAndGoToStation = PathPlanner.loadPath("RightSideLeaveAndGoToStation", new PathConstraints(KPPMaxVelocity, KPPMaxAcceleration));
  public static final PathPlannerTrajectory KLeftSideLeaveCommunity = PathPlanner.loadPath("LeftSideLeaveCommunity", new PathConstraints(KPPMaxVelocity, KPPMaxAcceleration));
  public static final PathPlannerTrajectory KLeftLeaveCommunityAndGoAway = PathPlanner.loadPath("LeftLeaveAndGoAway", new PathConstraints(KPPMaxVelocity, KPPMaxAcceleration));
  public static final PathPlannerTrajectory KPickUpRightSide = PathPlanner.loadPath("PickUpRightSide", new PathConstraints(KPPMaxVelocity, KPPMaxAcceleration));
  
  //sys id config numbers 
  public static final double ks = 0.20309;
  public static final double kv = 2.5574;
  public static final double ka = 0.38422;
  
  
  
  //More Lift Set Points
  // public static final double KliftHighScoreEncoderVal = 4.91;
  // public static final double KliftShelfGrabEncoderVal = 6980;
  // public static final double KLiftMidScoreEncoderVal = 3.60;
  
  public static final double KFlipperOffShelfPos = 138; // TBD
  public static final double KFlipperStowedPos = 25;  // TBD
  
  public static final double KFlipperSwivelSpeed = 0.1;
  public static final double KFlipperDeadzone = 0.5;

  public static final double KFlipperSwivelOffset = 223;
  // public static final double KFlipperSwivelOffset = 318;

  public static final double KFlipperMaxSpeed = 0.5;
  public static final double KFlipperMinSpeed = 0.15;

  
  
  public static final double KFlipperRollerIntakeSpeedCone = 0.5;
  public static final double KFlipperRollerOuttakeSpeedCone = -0.7;
  public static final double KFlipperRollerIntakeSpeedCube = -0.4;
  public static final double KFlipperRollerOuttakeSpeedCube = 0.3;
  
}
