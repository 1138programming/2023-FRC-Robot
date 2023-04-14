package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import com.revrobotics.CANSparkMaxLowLevel.FollowConfig.Config;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
  private CANSparkMax angleMotor;
  private CANSparkMax driveMotor;

  // magEncoder = absolute encoder to reset position of relative angle encoders
  private CANCoder canCoder;

  // Relative encoders are used for robot odometry and controlling speed/position
  private RelativeEncoder driveEncoder;
  private RelativeEncoder angleEncoder;

  private PIDController angleController;

  private double offset;

  private SimpleMotorFeedforward feedforward;
  private PIDController driveController;  

  public SwerveModule(int angleMotorID, int driveMotorID, int encoderPort, double offset, 
                      boolean driveMotorReversed, boolean angleMotorReversed) {
    angleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);
    driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);

    angleMotor.setIdleMode(IdleMode.kBrake);
    driveMotor.setIdleMode(IdleMode.kBrake);

    this.angleMotor.setInverted(angleMotorReversed);
    this.driveMotor.setInverted(driveMotorReversed);
    
    this.driveMotor.setSmartCurrentLimit(80);

    CANCoderConfiguration config = new CANCoderConfiguration();

    config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    config.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
    config.sensorDirection = false;
    config.magnetOffsetDegrees = offset;


    canCoder = new CANCoder(encoderPort);
    canCoder.configAllSettings(config);
    canCoder.setPositionToAbsolute();
    // canCoder.wait();  

    driveEncoder = driveMotor.getEncoder();
    angleEncoder = angleMotor.getEncoder();
    
    driveEncoder.setPositionConversionFactor(KDriveMotorRotToMeter);
    driveEncoder.setVelocityConversionFactor(KDriveMotorRPMToMetersPerSec);
    
    angleEncoder.setPositionConversionFactor(KAngleMotorRotToDeg);

    angleController = new PIDController(KAngleP, KAngleI, KAngleD);
    angleController.enableContinuousInput(-180, 180); // Tells PIDController that 180 deg is same in both directions


    setAbsoluteOffset(offset);
    // this.driveMotor.burnFlash();
    feedforward = new SimpleMotorFeedforward(ks, kv, ka);
    driveController = new PIDController(0.64442, 0, 0);
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("drive current" + driveMotor.getDeviceId(), driveMotor.getOutputCurrent());
    SmartDashboard.putString("posistion " + driveMotor.getDeviceId(), getPosition().toString());
    SmartDashboard.putNumber("encoder " + driveMotor.getDeviceId(), getAngleDeg());

    SmartDashboard.putNumber("mag deg " + driveMotor.getDeviceId(), getMagDeg());
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    // If no controller input, set angle and drive motor to 0
    if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
      angleMotor.set(0);
      driveMotor.set(0);
      return;
    }

    double angleMotorOutput;
    double driveMotorOutput;

    Rotation2d currentAngleR2D = getAngleR2D();
    desiredState = SwerveModuleState.optimize(desiredState, currentAngleR2D);

    // Angle calculation
    angleMotorOutput = angleController.calculate(getAngleDeg(), desiredState.angle.getDegrees());
    angleMotor.set(angleMotorOutput);

    // Drive calculation
    // driveMotorOutput = desiredState.speedMetersPerSecond / KPhysicalMaxDriveSpeedMPS;
    // SmartDashboard.putNumber("desired speed " + driveMotor.getDeviceId(), driveMotorOutput);
    // driveMotor.set(driveMotorOutput);

    double velocityOutput = driveController.calculate(getDriveEncoderVel(), desiredState.speedMetersPerSecond);
    

    driveMotorOutput = feedforward.calculate(desiredState.speedMetersPerSecond);
    // driveMotor.setVoltage(driveMotorOutput);
    driveMotor.setVoltage(driveMotorOutput + velocityOutput);
    SmartDashboard.putNumber("DRIVE SPEED " + driveMotor.getDeviceId(), driveMotorOutput);

    SmartDashboard.putNumber("wheel velocity " + driveMotor.getDeviceId(), getDriveEncoderVel());
    SmartDashboard.putNumber("wheel output current " + driveMotor.getDeviceId(), driveMotor.getOutputCurrent());
  }

  public void lockWheel() {
    double angleMotorOutput;
    if (angleMotor.getDeviceId() == KFrontLeftAngleID || angleMotor.getDeviceId() == KBackRightAngleID) {
      angleMotorOutput = angleController.calculate(getAngleDeg(), 45);
    }
    else {
      angleMotorOutput = angleController.calculate(getAngleDeg(), -45);
    }

    angleMotor.set(angleMotorOutput);
    driveMotor.set(0);
  }

  public SwerveModulePosition getPosition() {
    SwerveModulePosition position = new SwerveModulePosition(getDriveEncoderPos(), getAngleR2D());
    return position;
  }

  public void stop() {
    driveMotor.set(0);
    angleMotor.set(0);
  }

  public void resetRelEncoders() {
    driveEncoder.setPosition(0);
    angleEncoder.setPosition(getMagDeg());
  }

  public void setAbsoluteOffset(double offset) {
    this.offset = offset;
  }
  
  public double getAbsoluteOffset() {
    return offset;
  }

  // Drive Encoder getters
  public double getDriveEncoderPos() {
    return driveEncoder.getPosition();
  }
  public double getDriveEncoderVel() {
    return driveEncoder.getVelocity();
  }

  // Angle Encoder getters
  public double getMagDegRaw() {
    double pos = canCoder.getPosition();
    return pos;
  }
  public double getMagDeg() {
    return getMagDegRaw() % 360;
  }

  public double getAngleDeg() {
    // double angle = angleEncoder.getPosition() % 360;
    double angle = getMagDeg();
    return angle;
  }
  public Rotation2d getAngleR2D() {
    return Rotation2d.fromDegrees(getAngleDeg()); 
  }
}
