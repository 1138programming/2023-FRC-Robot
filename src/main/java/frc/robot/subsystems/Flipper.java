package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax; // Neos and 775
import com.revrobotics.RelativeEncoder;


import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.CANSparkMaxLowLevel.MotorType; // Covers Neos and 775 
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class Flipper extends SubsystemBase {
  

  private CANSparkMax flipperRoller;
  private TalonSRX flipperSwivel;  

  private CANCoder flipperCanCoder;
  private PIDController swivelController; 
  
  private boolean isConeMode;
  private double lastSwivelPos;

  public Flipper() 
  {
    flipperRoller = new CANSparkMax(KFlipperRollerMotor, MotorType.kBrushless);
    flipperSwivel = new TalonSRX(KFlipperSwivelMotor);
    flipperRoller.setInverted(false);
    flipperSwivel.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    flipperSwivel.setNeutralMode(NeutralMode.Brake);

    swivelController = new PIDController(KClawSwivelKP, KClawSwivelKI, KClawSwivelKd);
    
    CANCoderConfiguration config = new CANCoderConfiguration();

    lastSwivelPos = getEncoderPos();
    config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    config.magnetOffsetDegrees = KIntakeOffset;
    config.sensorDirection = true;


    flipperCanCoder = new CANCoder(KFlipperCanCoder);
    flipperCanCoder.configAllSettings(config);
    flipperCanCoder.setPositionToAbsolute();

    //scoringMode = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ClawSwivelPos", getEncoderPos());
    SmartDashboard.putNumber("flipper CanCoder", getFlipperCanCoderAbsPos());

    SmartDashboard.putNumber("Claw Default Calculation", swivelController.calculate(getEncoderPos(), lastSwivelPos));
  }

  // spin roller
  public void spinRoller(double speed) {
    flipperRoller.set(speed);
  }

  // spin swivel
  public void spinSwivel(double speed) {
    flipperSwivel.set(ControlMode.PercentOutput, speed);
    lastSwivelPos = getEncoderPos();  
  }

  public void spinSwivelToPos(Double position) {
    flipperSwivel.set(ControlMode.PercentOutput, swivelController.calculate(getEncoderPos()));
    lastSwivelPos = getEncoderPos();
  }

  public double getEncoderPos() {
    return flipperSwivel.getSelectedSensorPosition();
  }
  public double getFlipperCanCoderAbsPos() {
    return flipperCanCoder.getAbsolutePosition();
  }

  public void stop() {
    flipperRoller.set(0);
    flipperSwivel.set(ControlMode.PercentOutput, 0);
    // double swivelSpeed = swivelController.calculate(getEncoderPos(), lastSwivelPos);
    // if (swivelSpeed > 0.05) {
    //   swivelSpeed = 0.05;
    // }
    // else if (swivelSpeed < -0.05) {
    //   swivelSpeed = -0.05;
    // }
    // swivel.set(ControlMode.PercentOutput, swivelSpeed);
  } 
}
