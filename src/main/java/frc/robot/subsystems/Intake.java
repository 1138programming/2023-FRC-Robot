// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class Intake extends SubsystemBase {
  private TalonSRX swivel;
  // "spaghetti" refers to the spaghetti-looking motors that actually do the intaking.
  private TalonSRX spaghetti;

  private CANCoder intakeSwivelCanCoder; 

  private PIDController intakeController;
  private SlewRateLimiter intakeSlewRateLimiter = new SlewRateLimiter(0.5);

  private DigitalInput intakeTopLimit;
  private DigitalInput intakeBottomLimit;

  CANCoderConfiguration config;

  AddressableLED ledStrip;
  AddressableLEDBuffer ledBuffer;

  private boolean intakeMode;
  private boolean defenseMode = false;

  private double finalCancoderVal;
  private double lastSwivelPos;

  private SlewRateLimiter swivelLimiter;

  public Intake() {
    SmartDashboard.putNumber("IntakeSwivelPid P", KIntakeP);
    SmartDashboard.putNumber("IntakeSwivelPid I", KIntakeI);
    SmartDashboard.putNumber("IntakeSwivelPid D", KIntakeD);

    spaghetti = new TalonSRX(KSpaghettiIntakeId);
    swivel = new TalonSRX(KSwivelIntakeId);

    swivel.setNeutralMode(NeutralMode.Brake);
    spaghetti.setNeutralMode(NeutralMode.Coast);
    
    swivel.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    spaghetti.setInverted(true); 

    intakeController = new PIDController(KIntakeP, KIntakeI, KIntakeD);
    // intakeController.

    intakeBottomLimit = new DigitalInput(KIntakeBottomLimitId);
    intakeTopLimit = new DigitalInput(KIntakeTopLimitId);

    ledStrip = new AddressableLED(KLEDPort);
    ledBuffer = new AddressableLEDBuffer(KLEDBuffer);

    ledStrip.setLength(ledBuffer.getLength());
    ledStrip.setData(ledBuffer);
    ledStrip.start();
    
    swivelLimiter = new SlewRateLimiter(1);
    
    config = new CANCoderConfiguration();
    
    config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    config.magnetOffsetDegrees = KIntakeOffset;
    config.sensorDirection = true;
    setConeMode();
    
    intakeSwivelCanCoder = new CANCoder(KIntakeCanCoder);
    intakeSwivelCanCoder.configAllSettings(config);
    intakeSwivelCanCoder.setPositionToAbsolute();
    
    finalCancoderVal = 0;
    lastSwivelPos = 20;
    intakeMode = KConeMode;
  }

  @Override 
  public void periodic() {
    // intakeController.setP(SmartDashboard.getNumber("IntakeSwivelPid P", KIntakeP));
    // intakeController.setI(SmartDashboard.getNumber("IntakeSwivelPid I", KIntakeI));
    // intakeController.setD(SmartDashboard.getNumber("IntakeSwivelPid D", KIntakeD));


    SmartDashboard.putBoolean("Mode", intakeMode);
    SmartDashboard.putNumber("Swivel Cancoder", getSwivelEncoder());
    SmartDashboard.putNumber("raw", intakeSwivelCanCoder.getAbsolutePosition());
    SmartDashboard.putNumber("Raw Swivel CanCoder", getSwivelEncoderRaw());
    SmartDashboard.putBoolean("limit INTAKE", getTopLimitSwitch());

    if (getTopLimitSwitch()) {
      setIntakeEncoder(0);
      resetSwivelEncoder();
    }
    
  }

  /**
   * Spins the "spaghetti" motors (the spinners in the intake)
   */
  public void spaghettiSpin() {
    // if (intakeMode) {
    //   spaghetti.set(ControlMode.PercentOutput, KIntakeConeSpaghettitSpeed);
    // }
    // else if (!intakeMode) {
    //   spaghetti.set(ControlMode.PercentOutput, KIntakeCubeSpaghettitSpeed);
    // }
    spaghetti.set(ControlMode.PercentOutput, KIntakeCubeSpaghettitSpeed);
  }
  
  public void spaghettiSpinReverse(double speed) {
    spaghetti.set(ControlMode.PercentOutput, -speed);
  }
  public void spaghettiSpinReverse() {
    if (intakeMode) {
      spaghetti.set(ControlMode.PercentOutput, -KIntakeConeSpaghettitSpeed);
    }
    else if (!intakeMode) {
      spaghetti.set(ControlMode.PercentOutput, -KIntakeCubeSpaghettitSpeed);
    }
  }

  public void setLEDToColor(int R, int G, int B) {
    // ledsOff();
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, R, G, B);
    }
    ledStrip.setData(ledBuffer);
    // ledStrip.
    // ledStrip.start();
  }


  public void ledsOff() {
    ledStrip.stop();
  }
  
  public void setCubeMode() {
    intakeMode = KCubeMode;
    defenseMode = false; 

    // set the led strip to purple
    setLEDToColor(119, 11, 219);
  }

  public void setConeMode() {
    intakeMode = KConeMode;
    defenseMode = false; 
    
    // yellow
    setLEDToColor(150, 150, 0);
  }

  public boolean getObjectMode() {
    return intakeMode;
  }

  // get the operating mode of the intake
  public boolean isConeMode() {
    return intakeMode; 
  }

  public void swivelSpinToPos(double setPoint) {
    
    double speed = -intakeController.calculate(getSwivelEncoder(), setPoint);
    // if (speed > 0.6) {
    //   speed = 0.6;
    // }
    // if (speed < -0.6) {
    //   speed = -0.6;
    // }
    SmartDashboard.putNumber("swivel speed!", speed);
    moveSwivel(speed);
  }

  /**
   * Moves the swivel at a certain speed
   * @param speed
   */
  public void moveSwivel(double speed) {
    // if (intakeSwivelCanCoder.)
    // May need a rate limiter...
    if (speed > 0 && getTopLimitSwitch()) {
      swivel.set(ControlMode.PercentOutput, 0);
    }
    else {
      swivel.set(ControlMode.PercentOutput, speed);
    }
    lastSwivelPos = getSwivelEncoder();
  }

  public void setIntakeEncoder(double position) {
    swivel.setSelectedSensorPosition(position);
  }
  
  public void resetSwivelEncoder() {
    finalCancoderVal  = getSwivelEncoder() % (360 * 14/32);
  }
  public double getSwivelEncoder() {
    if (getSwivelEncoderRaw() < 50 && finalCancoderVal >= 100) {
      finalCancoderVal = 360 * 14/34 + getSwivelEncoderRaw();
    }
    else {
      finalCancoderVal = getSwivelEncoderRaw();
    }
    return finalCancoderVal;
  }

  public double getSwivelEncoderRaw() {
    return intakeSwivelCanCoder.getAbsolutePosition() * 14/34;
  }

  public boolean getTopLimitSwitch() {
    return intakeTopLimit.get();
  }
  
  public void setLastEncoderPos(double lastEncoderPos) {
    lastSwivelPos = lastEncoderPos;
  }

  public void resetPIDController() {
    // intakeController.reset();
  }

  public void intakeStop() {
    swivel.set(ControlMode.PercentOutput, -intakeController.calculate(getSwivelEncoder(), lastSwivelPos));
    // swivel.set(ControlMode.PercentOutput, 0);
    spaghetti.set(ControlMode.PercentOutput, 0);
  }
}