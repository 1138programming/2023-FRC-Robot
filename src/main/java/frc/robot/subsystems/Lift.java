package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax; // Neos and 775
import com.revrobotics.CANSparkMaxLowLevel.MotorType; // Covers Neos and 775 

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import 
import com.revrobotics.CANSparkMax.IdleMode;

public class Lift extends SubsystemBase {
  private CANSparkMax lift;

  private TalonFX flipperSwivel;
  private CANSparkMax flipperRoller;

  private CANCoder flipperCanCoder;
  private CANCoderConfiguration config;
  private double finalCancoderVal;

  private Encoder liftShaftEncoder;
  private DigitalInput liftSwitch;

  private PIDController liftControl;
  private PIDController flipperController;

  private boolean objectMode;

  public Lift()
  {
    flipperSwivel = new TalonFX(KFlipperSwivelMotorID);
    lift = new CANSparkMax(KLiftMotorID, MotorType.kBrushless);
    flipperRoller = new CANSparkMax(KFlipperRollerMotorID, MotorType.kBrushless);
   
    flipperSwivel.setNeutralMode(NeutralMode.Brake);
    lift.setIdleMode(IdleMode.kBrake);
    flipperRoller.setIdleMode(IdleMode.kBrake);

    liftControl = new PIDController(KLiftP, KLiftI, KLiftD);
    flipperController = new PIDController(KFlipperP, KFlipperI, KFlipperD);
    
    liftShaftEncoder = new Encoder(KLiftEncoderA, KLiftEncoderB);

    config = new CANCoderConfiguration();

    config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    config.magnetOffsetDegrees = KFlipperSwivelOffset;
    config.sensorDirection = true;

    flipperCanCoder = new CANCoder(KFlipperCanCoder);
    flipperCanCoder.configAllSettings(config);
    flipperCanCoder.setPositionToAbsolute();

    flipperSwivel.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    liftSwitch = new DigitalInput(KScoringBottomLimitSwitch);

    finalCancoderVal = 5;

    objectMode = KConeMode;
  }

  @Override
  public void periodic() {
    if (getBottomLimitSwitch()) {
      setLiftEncoderPos(0);
    }

    SmartDashboard.putNumber("lift encoder", getLiftPos());
    SmartDashboard.putNumber("lift speed", lift.get());
    SmartDashboard.putBoolean("lift limit switch", getBottomLimitSwitch());
    SmartDashboard.putNumber("Lift Encoder FOR REAL", liftShaftEncoder.getDistance());

    SmartDashboard.putNumber("flipper encoder", getFlipperPos());
    SmartDashboard.putNumber("flipper CanCoder", getFlipperPosRaw());
    SmartDashboard.putNumber("flipper CanCoder RAW", flipperCanCoder.getAbsolutePosition());
    
    SmartDashboard.putBoolean("object mode", objectMode);
  }

  public void setLiftPos(double setPoint) {
    double liftSpeed;
    liftSpeed = liftControl.calculate(getLiftPos(), setPoint);
    
    moveLift(liftSpeed);
  }


  public void moveLift(double speed) {
    if (getBottomLimitSwitch() && speed < 0) {
      speed = 0;
    }
    if (liftShaftEncoder.getDistance() >= KLiftHighPos && speed > 0) {
      speed = 0;
    }
    lift.set(speed);
  }

  // flipper
  public void flipToPos(double setPoint) {
    moveFlipperSwivel(-flipperController.calculate(getFlipperPos(), setPoint));
  }

  public void moveFlipperSwivel(double speed) {
    if (speed > KFlipperMaxSpeed) {
      speed = KFlipperMaxSpeed;
    } else if (speed < -KFlipperMaxSpeed) {
      speed = -KFlipperMaxSpeed;
    }

    flipperSwivel.set(ControlMode.PercentOutput, speed);
  }

  public double getFlipperPos(){
    if (getFlipperPosRaw() < 50 && finalCancoderVal >= 100) {
      finalCancoderVal = 360 * KFlipperCanCoderRatio + getFlipperPosRaw();
    } else {
      finalCancoderVal = getFlipperPosRaw();
    }
    return finalCancoderVal;
  }
  public double getFlipperPosRaw() {
    return flipperCanCoder.getAbsolutePosition() * KFlipperCanCoderRatio;
  }

  // lift
  public double getLiftPos() {
    return liftShaftEncoder.getDistance();
  }
  public void setLiftEncoderPos(double pos) {
    liftShaftEncoder.reset();
  }

  // Roller
  public void spinRoller(double speed) {
    flipperRoller.set(speed);
  }
  
  public void intakeRoller() {
    if (objectMode == KConeMode) {
      flipperRoller.set(KFlipperRollerIntakeSpeedCone);
    } else if (objectMode == KCubeMode) {
      flipperRoller.set(KFlipperRollerIntakeSpeedCube);
    }
  }

  public void outtakeRoller() {
    if (objectMode == KConeMode) {
      flipperRoller.set(KFlipperRollerOuttakeSpeedCone);
    } else if (objectMode == KCubeMode) {
      flipperRoller.set(KFlipperRollerOuttakeSpeedCube);
    }
  }
  public boolean getBottomLimitSwitch() {
    return !liftSwitch.get();
  }

  public void setCubeMode() {
    objectMode = KCubeMode;
  }
  public void setConeMode() {
    objectMode = KConeMode;
  }

  public void stopRollers() {
    flipperRoller.set(0);
  }

  public void stop() {
    lift.set(0);
    flipperRoller.set(0);
    flipperSwivel.set(ControlMode.PercentOutput, 0);
  }
}