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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
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
    
  public SwerveModule(int angleMotorID, int driveMotorID, int encoderPort, double offset, 
                      boolean driveMotorReversed, boolean angleMotorReversed) {
    angleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);
    driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);

    angleMotor.setIdleMode(IdleMode.kBrake);
    driveMotor.setIdleMode(IdleMode.kBrake);

    this.angleMotor.setInverted(angleMotorReversed);
    this.driveMotor.setInverted(driveMotorReversed);

    // this.magEncoder = magEncoder;
    CANCoderConfiguration config = new CANCoderConfiguration();

    config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    // config.enableOptimizations = false;
    // config.
    config.sensorDirection = false;
    // config.


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
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    SmartDashboard.putNumber("encoder " + driveMotor.getDeviceId(), getDriveEncoderPos());
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
    driveMotorOutput = desiredState.speedMetersPerSecond / KPhysicalMaxDriveSpeedMPS;

    driveMotor.set(driveMotorOutput);
  }

  public SwerveModulePosition getPosition() {
    SwerveModulePosition position = new SwerveModulePosition(getDriveEncoderPos(), getAngleR2D());
    SmartDashboard.putString("posistion " + driveMotor.getDeviceId(), position.toString());
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
    double pos = canCoder.getPosition() + offset;
    return pos;
  }
  public double getMagDeg() {
    return getMagDegRaw() % 360;
  }

  public double getAngleDeg() {
    return angleEncoder.getPosition() % 360;
  }
  public Rotation2d getAngleR2D() {
    return Rotation2d.fromDegrees(getAngleDeg()); 
  }
}
