package frc.robot.subsystems;

import static frc.robot.Constants.*;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
// import 
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxAbsoluteEncoder;

public class Lift extends SubsystemBase {
  private CANSparkMax lift;
  private CANSparkMax innerLift;
  private CANSparkMax flipper;

  private PIDController liftControl;
  private PIDController liftDownController;
  private PIDController innerLiftControl;
  private PIDController flipperController;

  private SlewRateLimiter liftLimiter;

  private RelativeEncoder liftEncoder;
  private RelativeEncoder innerLiftEncoder;
  private RelativeEncoder flipperEncoder;

  private DigitalInput liftSwitch;

  private double innerLiftSpeed = 0;
  
  public Lift()
  {
    flipper = new CANSparkMax(KFlipperMotor, MotorType.kBrushless);
    lift = new CANSparkMax(KLiftMotor, MotorType.kBrushless);
    // innerLift = new CANSparkMax(KInnerLiftMotor, MotorType.kBrushed);

    innerLift.setIdleMode(IdleMode.kBrake);
    flipper.setIdleMode(IdleMode.kBrake);
    lift.setIdleMode(IdleMode.kBrake);
    
    liftControl = new PIDController(0.05, 0.001, 0.0001);
    liftDownController = new PIDController(0.01, 0, 0);
    innerLiftControl = new PIDController(0.0005, KInnerLiftI, KInnerLiftD);
    flipperController = new PIDController(KFlipperP, KFlipperI, KFlipperD);

    liftLimiter = new SlewRateLimiter(1);
    // innerLift.
    
    innerLiftEncoder = innerLift.getEncoder(com.revrobotics.SparkMaxRelativeEncoder.Type.kQuadrature,1);
    liftEncoder = lift.getEncoder();
    flipperEncoder = flipper.getEncoder();

    // innerLiftEncoder.setPosition(0);
    innerLiftEncoder.setPositionConversionFactor(0.1);
    innerLiftEncoder.setPosition(2000);

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
    SmartDashboard.putNumber("UPPER LIFT encoder", getInnerLiftPos());

  }
  public void setLiftPos (double setPoint) {
    double liftSpeed;
    if (setPoint > getLiftPos()) {
      liftSpeed = liftControl.calculate(getLiftPos(), setPoint);
    }
    else {
      liftSpeed = -0.3;
      // liftSpeed = liftDownController.calculate(getLiftPos(), setPoint);
    }
    if (liftSpeed > 0.2) {
      liftSpeed = 0.2;
    }
    lift.set(liftSpeed);
  }
  public void moveLift(double speed) {
    if (getBottomLimitSwitch() && speed < 0) {
      lift.set(0);
    }
    // else if (speed < 0) {
    //   lift.set(liftLimiter.calculate(speed));
    // }
    else {
      lift.set(liftLimiter.calculate(speed));
    }
  }

  public void setInnerLiftPos(double setPoint) {
    
    innerLiftSpeed = -innerLiftControl.calculate(innerLiftEncoder.getPosition(),setPoint);
    if (innerLiftSpeed > 0.3) {
      innerLiftSpeed = 0.3;
    }
    innerLift.set(innerLiftSpeed);

    // if (setPoint < getLiftPos()) {
    // }
    // else {
    //   innerLift.set(innerLiftControl.calculate(innerLiftEncoder.getPosition(),setPoint));
    // }
  }
  public void moveInnerLift(double speed) {
    innerLiftSpeed = -speed;
    innerLift.set(-speed);
  }
  
  public void flipToPos(double setPoint) {
    moveFlipper(flipperController.calculate(getFlipperPos(), setPoint));

    // innerLift.set(-0.05);
  }
  public void moveFlipper(double speed) {
    flipper.set(speed);
  }
  
  public double getLiftPos() {
    return liftEncoder.getPosition();
  }
  public void setLiftEncoderPos(double pos) {
    liftEncoder.setPosition(pos);
  }

  public double getInnerLiftPos() {
    return innerLiftEncoder.getPosition();
  }
  public double getFlipperPos(){
    return flipperEncoder.getPosition();
  }

  public boolean getBottomLimitSwitch() {
    return !liftSwitch.get();
  }

  public void stop() {
      lift.set(0);
      innerLift.set(-0.05);
      flipper.set(0);
  }       
}
