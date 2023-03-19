package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax; // Neos and 775
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType; // Covers Neos and 775 
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.SparkMaxAbsoluteEncoder;
// import 

public class Lift extends SubsystemBase {
  private CANSparkMax lift;
  private CANSparkMax innerLift;
  private CANSparkMax flipper;

  private PIDController liftControl;
  private PIDController liftDownController;

  private PIDController innerLiftControl;
  private PIDController flipperController;

  private RelativeEncoder liftEncoder;
  private RelativeEncoder innerLiftEncoder;
  private RelativeEncoder flipperEncoder;

  private DigitalInput liftSwitch;
  
  public Lift()
  {
    lift = new CANSparkMax(KLiftMotor, MotorType.kBrushless);
    liftControl = new PIDController(0.05, 0.001, 0.0001);
    liftDownController = new PIDController(0.03, 0, 0);
    liftEncoder = lift.getEncoder();

    innerLift = new CANSparkMax(KInnerLiftMotor, MotorType.kBrushless);
    innerLiftControl = new PIDController(KInnerLiftP, KInnerLiftI, KInnerLiftD);
    innerLiftEncoder = innerLift.getEncoder();
    // flipperEncoder = new 
    // flipperEncoder = flipper.getAbsoluteEncoder(Type.kDutyCycle);
    
    flipperController = new PIDController(KFlipperP, KFlipperI, KFlipperD);

    liftEncoder.setPosition(0);

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
  }
  public void setLiftPos (double setPoint) {
    double liftSpeed;
    if (setPoint > getLiftPos()) {
      liftSpeed = liftControl.calculate(getLiftPos(), setPoint);
    }
    else {
      liftSpeed = liftDownController.calculate(getLiftPos(), setPoint);
    }
    lift.set(liftSpeed);
  }
  public void moveLift(double speed) {
    if (getBottomLimitSwitch() && speed < 0) {
      lift.set(0);
    }
    else {
      lift.set(speed);
    }
  }

  public void setInnerLiftPos(double setPoint) {
    if (setPoint < getLiftPos()) {
      innerLift.set(innerLiftControl.calculate(innerLiftEncoder.getPosition(),setPoint));
    }
    else {
      innerLift.set(innerLiftControl.calculate(innerLiftEncoder.getPosition(),setPoint));
    }
  }
  public void moveInnerLift(double speed) {
    innerLift.set(speed);
  }
  
  public void flipToPos(double setPoint) {
    moveFlipper(flipperController.calculate(getFlipperPos(), setPoint));
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
      innerLift.set(0);
      flipper.set(0);
  }       
}
