package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax; // Neos and 775
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.CANSparkMaxLowLevel.MotorType; // Covers Neos and 775 
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxAnalogSensor.Mode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
// import 
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxAbsoluteEncoder;

public class Lift extends SubsystemBase {
  private CANSparkMax lift;
 
  private CANSparkMax flipperSwivel;

  private CANCoder flipperCanCoder;
  private CANCoderConfiguration config;

  private PIDController liftControl;
  private PIDController liftDownController;
 
  private PIDController flipperController;

  private SlewRateLimiter liftLimiter;

  private RelativeEncoder liftEncoder;
  private Encoder liftEncoder2;
 
  private RelativeEncoder flipperEncoder;

  private DigitalInput liftSwitch;

 
  
  public Lift()
  {
    flipperSwivel = new CANSparkMax(KFlipperSwivelMotor, MotorType.kBrushless);
    lift = new CANSparkMax(KLiftMotor, MotorType.kBrushless);
   
    flipperSwivel.setIdleMode(IdleMode.kBrake);
    lift.setIdleMode(IdleMode.kBrake);
    
    liftControl = new PIDController(0.05, 0.001, 0.0001);
    liftDownController = new PIDController(0.01, 0, 0);
    
    flipperController = new PIDController(KFlipperP, KFlipperI, KFlipperD);

    liftLimiter = new SlewRateLimiter(1);
    
    liftEncoder = lift.getEncoder();

    liftEncoder2 = new Encoder(KLiftEncoderA, KLiftEncoderB);


    flipperEncoder = flipperSwivel.getEncoder();

    config = new CANCoderConfiguration();

    config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    config.magnetOffsetDegrees = KIntakeOffset;
    config.sensorDirection = true;


    flipperCanCoder = new CANCoder(KFlipperCanCoder);
    flipperCanCoder.configAllSettings(config);
    flipperCanCoder.setPositionToAbsolute();
    // ledStrip.start();

    // flipperEncoder.setPosition(0);

    liftSwitch = new DigitalInput(KScoringBottomLimitSwitch);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("lift encoder", getLiftPos());
    SmartDashboard.putNumber("lift speed", lift.get());
    if (getBottomLimitSwitch()) {
      setLiftEncoderPos(0);
    }
    SmartDashboard.putBoolean("lift limit switch", getBottomLimitSwitch());

    SmartDashboard.putNumber("flipper encoder", getFlipperPos());
   
    SmartDashboard.putNumber("flipper CanCoder", getFlipperCanCoderAbsPos());
    
    SmartDashboard.putNumber("Lift Encoder FOR REAL", liftEncoder2.getDistance());
    

  }
  public void setLiftPos (double setPoint) {
    double liftSpeed;
    if (setPoint > getLiftPos()) {
      liftSpeed = liftControl.calculate(getLiftPos(), setPoint);
    }
    else {
      liftSpeed = -0.2;
      // liftSpeed = liftDownController.calculate(getLiftPos(), setPoint);
    }
    if (liftSpeed > 0.2) {
      liftSpeed = 0.2;
    }
    moveLift(liftSpeed);
  }
  public void moveLift(double speed) {
    if (getBottomLimitSwitch() && speed < 0) {
      lift.set(0);
    }
    lift.set(speed);
    
    // else if (speed < 0) {
    //   lift.set(liftLimiter.calculate(speed));
    // }
    // else {
    //   lift.set(liftLimiter.calculate(speed));
    // }
  }

 

  // flipper
  public void flipToPos(double setPoint) {
    moveFlipper(flipperController.calculate(getFlipperPos(), setPoint));

  }
  public void moveFlipper(double speed) {
    flipperSwivel.set(speed);
  }

  public double getFlipperPos(){
    return flipperEncoder.getPosition();
  }
  public double getFlipperCanCoderAbsPos() {
    return flipperCanCoder.getAbsolutePosition();
  }

  // lift
  public double getLiftPos() {
    return liftEncoder.getPosition();
  }
  public void setLiftEncoderPos(double pos) {
    liftEncoder.setPosition(pos);
  }


  

  public boolean getBottomLimitSwitch() {
    return !liftSwitch.get();
  }

  public void stop() {
      lift.set(0);
      
      flipperSwivel.set(0);
  }       
}
