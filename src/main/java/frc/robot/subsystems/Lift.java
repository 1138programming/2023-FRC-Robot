package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax; // Neos and 775
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType; // Covers Neos and 775 
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
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
  private RelativeEncoder liftEncoder;
  private PIDController innerLiftControl;
  private RelativeEncoder innerLiftEncoder;
  private SparkMaxAbsoluteEncoder flipperEncoder;
  
  private PIDController flipperController;
  
  public Lift()
  {
    lift = new CANSparkMax(KLiftMotor, MotorType.kBrushless);
    liftControl = new PIDController(KLiftP, KLiftI, KLiftD);
    liftEncoder = lift.getEncoder();

    innerLift = new CANSparkMax(KInnerLiftMotor, MotorType.kBrushless);
    innerLiftControl = new PIDController(KInnerLiftP, KInnerLiftI, KInnerLiftD);
    innerLiftEncoder = innerLift.getEncoder();
    // flipperEncoder = new 
    flipperEncoder = flipper.getAbsoluteEncoder(Type.kDutyCycle);
    
    flipperController = new PIDController(KFlipperP, KFlipperI, KFlipperD);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("FlipperEncoder", flipperEncoder.getPosition()); 
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

public void moveInnerLift(double setPoint) {
    innerLift.set(innerLiftControl.calculate(innerLiftEncoder.getPosition(),setPoint));
}
public double getInnerLiftPos() {
    return innerLiftEncoder.getPosition();
}
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
