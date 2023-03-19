package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax; // Neos and 775
import com.revrobotics.EncoderType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.CANSparkMaxLowLevel.MotorType; // Covers Neos and 775 
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxAnalogSensor.Mode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
// import 

import com.revrobotics.SparkMaxAbsoluteEncoder;

public class Lift extends SubsystemBase {
  private CANSparkMax lift;
  private CANSparkMax innerLift;

  private CANSparkMax flipper;

  private PIDController liftControl;
  private RelativeEncoder liftEncoder;
  private PIDController innerLiftControl;
  private RelativeEncoder innerLiftEncoder;
  private DutyCycleEncoder coolEncoder;
  private AbsoluteEncoder abs;
  private RelativeEncoder flipperEncoder;
  
  private PIDController flipperController;
  
  public Lift()
  {
    lift = new CANSparkMax(KLiftMotor, MotorType.kBrushless);
    liftControl = new PIDController(KLiftP, KLiftI, KLiftD);
    liftEncoder = lift.getEncoder();

    innerLift = new CANSparkMax(16, MotorType.kBrushed);
    // innerLiftControl = new PIDController(KInnerLiftP, KInnerLiftI, KInnerLiftD);
    //  innerLiftEncoder = new DutyCycleEncoder(8);
    innerLiftEncoder = innerLift.getEncoder(com.revrobotics.SparkMaxRelativeEncoder.Type.kQuadrature,1);
    // innerLiftEncoder = innerLift.get
    coolEncoder = new DutyCycleEncoder(8);

    // innerLiftEncoder.setPositionConversionFactor(1);

    flipper = new CANSparkMax(KFlipperMotor, MotorType.kBrushless);
    flipperEncoder = flipper.getEncoder();
    flipperController = new PIDController(KFlipperP, KFlipperI, KFlipperD);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("FlipperEncoder", flipperEncoder.getPosition()); 
    // moveInnerLift(0.5);
    SmartDashboard.putNumber("DUTY", coolEncoder.get());
    SmartDashboard.putNumber("DUTY ABSOLUTE", coolEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("INNER LIFT ENCODER", innerLiftEncoder.getPosition()); 
    SmartDashboard.putNumber("Inner lift encoder conversion factor", innerLiftEncoder.getPositionConversionFactor());

  }
  public void moveFlipper(double speed) {
    flipper.set(speed);
  }
  public void moveLift(double setPoint) {
    lift.set(liftControl.calculate(liftEncoder.getPosition(),setPoint));
  }
  public double getLiftPos() {
    return liftEncoder.getPosition();
  }

public void moveInnerLift(double speed) {         
  innerLift.set(speed);
    // innerLift.set(innerLiftControl.calculate(innerLiftEncoder.getPosition(),setPoint));
}
// public double getInnerLiftPos() {
//     return innerLiftEncoder.getPosition();
// }
  public double getFlipperPos(){
    return flipperEncoder.getPosition();
  }


  public void flipToPos(double setPoint) {
      moveFlipper(flipperController.calculate(getFlipperPos(), setPoint));
  }



  public void stop() {
      lift.set(0);
      innerLift.set(0);
      flipper.set(0);
  }       
}
