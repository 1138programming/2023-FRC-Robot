// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;


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
  // public static final double KMaxAngularSpeed = Math.PI * 2;
  private static final double KBaseCircumference = 2.493518;
  public static final double KMaxAngularSpeed = 6;
  // KPhysicalMaxDriveSpeedMPS / KBaseCircumference;
  
  
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
  public static final double KBalanceP = 0.0055;
  public static final double KBalanceI = 0;
  public static final double KBalanceD = 0.001;
  
  // Pathplanner trajectories
  public static final PathPlannerTrajectory KBlue1 = PathPlanner.loadPath("Blue1", new PathConstraints(KPPMaxVelocity, KPPMaxAcceleration));
  public static final PathPlannerTrajectory KBlue1Part2 = PathPlanner.loadPath("Blue1 Part 2", new PathConstraints(KPPMaxVelocity, KPPMaxAcceleration));
  // public static final PathPlannerTrajectory KLeftSideLeaveCommunity = PathPlanner.loadPath("blue1", new PathConstraints(KPPMaxVelocity, KPPMaxAcceleration));
  
  //sys id config numbers 
  public static final double ks = 0.20309;
  public static final double kv = 2.5574;
  public static final double ka = 0.38422;
}