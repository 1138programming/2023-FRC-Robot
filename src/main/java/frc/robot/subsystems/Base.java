package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

  private boolean defenseMode = false;
  
  private double driveSpeedFactor;
  private double rotSpeedFactor;

  private double xyP = 0;
  private double rotP = 0;

  private boolean fastSpeedButton;
  private boolean slowSpeedButton;
  
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
    driveSpeedFactor = KBaseDriveMidPercent;
    rotSpeedFactor = KBaseRotMidPercent;

    fastSpeedButton = false;
    slowSpeedButton = false;

    SmartDashboard.putNumber("X and Y PID", 0);
    SmartDashboard.putNumber("rot P", 0);
  }

  // public double driv

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, double maxDriveSpeedMPS) {
    xSpeed *= maxDriveSpeedMPS;
    ySpeed *= maxDriveSpeedMPS;
    rot *= KMaxAngularSpeed * getRotSpeedFactor();
    
    //feeding parameter speeds into toSwerveModuleStates to get an array of SwerveModuleState objects
    SwerveModuleState[] states =
      kinematics.toSwerveModuleStates(
        fieldRelative
          ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getHeading())
          : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(states, KPhysicalMaxDriveSpeedMPS);

    if (defenseMode) {
      lockWheels();
    }
    else {
      //setting module states, aka moving the motors
      frontLeftModule.setDesiredState(states[0]);
      frontRightModule.setDesiredState(states[1]);
      backLeftModule.setDesiredState(states[2]);
      backRightModule.setDesiredState(states[3]);
    }
  }

  public void lockWheels() {
    frontLeftModule.lockWheel(); 
    frontRightModule.lockWheel();
    backLeftModule.lockWheel();
    backRightModule.lockWheel();
  }
  
  public void resetOdometry(Pose2d pose) {
    resetAllRelEncoders();
    this.pose = pose;
    
    odometry.resetPosition(getHeading(), getPositions(), pose);
  }

  public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    return new SequentialCommandGroup(
         new InstantCommand(() -> {
           // Reset odometry for the first path you run during auto
           if(isFirstPath){
              resetOdometry(traj.getInitialHolonomicPose());
           }
         }),
         new PPSwerveControllerCommand(
             traj,
             this::getPose, // Pose supplier
             this.kinematics, // SwerveDriveKinematics
             new PIDController(xyP, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
             new PIDController(xyP, 0, 0), // Y controller (usually the same values as X controller)
             new PIDController(rotP, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
             this::setModuleStates, // Module states consumer
             false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
             this // Requires this drive subsystem
         )
     );
  }

  // public Command followTrajectoryWPILib(Trajectory traj) {
  //   // return ne Se
  // }

  public Pose2d getPose() {
    return pose;
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    frontLeftModule.setDesiredState(desiredStates[0]);
    frontRightModule.setDesiredState(desiredStates[1]);
    backLeftModule.setDesiredState(desiredStates[2]);
    backRightModule.setDesiredState(desiredStates[3]);
  }

  // recalibrates gyro offset
  public void resetGyro() {
    gyro.reset(); 
    gyro.setAngleAdjustment(0);
  }

  public void resetGyro(double gyroOffset) {
    gyro.reset();
    gyro.setAngleAdjustment(gyroOffset);
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
    // return -gyro.getRoll();
    // return -gyro.getPitch();
    return -gyro.getAngle();
    // return 0;
  }

  public double getRoll() {
    return gyro.getRoll();
  }
  public double getPitch() {
    return gyro.getPitch();
  }

  public void resetOdometry() {
    resetAllRelEncoders();
    pose = new Pose2d();
    
    odometry.resetPosition(getHeading(), getPositions(), pose);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("pitch", getPitch());
    // SmartDashboard.putBoolean("DEFENSE MODE", getDefenseMode());
    // SmartDashboard.putNumber("Gyro", getHeadingDeg());

    // SmartDashboard.putNumber("Front left module", frontLeftModule.getAngleDeg());
    // SmartDashboard.putNumber("Front right module", frontRightModule.getAngleDeg());
    // SmartDashboard.putNumber("Back left module", backLeftModule.getAngleDeg());
    // SmartDashboard.putNumber("Back right module", backRightModule.getAngleDeg());

    // SmartDashboard.putNumber("front left mag", frontLeftModule.getMagDeg());
    // SmartDashboard.putNumber("front right mag", frontRightModule.getMagDeg());
    // SmartDashboard.putNumber("back left mag", backLeftModule.getMagDeg());
    // SmartDashboard.putNumber("back right mag", backRightModule.getMagDeg());

    SmartDashboard.putString("odometry pose", odometry.getPoseMeters().toString());

    odometry.update(getHeading(), getPositions());
    pose = odometry.getPoseMeters();
  }

  public double getDriveSpeedFactor()
  {
    return driveSpeedFactor;
  }
  public void setDriveSpeedFactor(double speedFactor)
  {
    driveSpeedFactor = speedFactor;
  }

  public void setRotSpeedFactor(double speedFactor)
  {
    rotSpeedFactor = speedFactor;
  }
  public double getRotSpeedFactor() {
    return rotSpeedFactor;
  }

  public boolean getDefenseMode() {
    return defenseMode;
  }
  public void setDefenseMode(boolean defenseMode) {
    this.defenseMode = defenseMode;
  }
}
  
  