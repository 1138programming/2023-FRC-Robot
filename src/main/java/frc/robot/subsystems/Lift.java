package frc.robot.subsystems;

import static frc.robot.Constants.*;
import com.revrobotics.CANSparkMax; // Neos and 775
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType; // Covers Neos and 775 
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lift extends SubsystemBase {
  private CANSparkMax lift;
  private CANSparkMax flipper;
  private PIDController liftControl;
  private RelativeEncoder liftEncoder;
  private RelativeEncoder flipperEncoder;
  private PIDController flipperController;
  
  public Lift()
  {
    lift = new CANSparkMax(KLiftMotor, MotorType.kBrushless);
    liftControl = new PIDController(KLiftP, KLiftI, KLiftD);
    liftEncoder = lift.getEncoder();
    flipper = new CANSparkMax(KFlipperMotor, MotorType.kBrushless);
    flipperEncoder = flipper.getEncoder();
    flipperController = new PIDController(KFlipperP, KFlipperI, KFlipperD);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
  public double getFlipperPos(){
    return flipperEncoder.getPosition();
  }


  public void flipToPos(double setPoint) {
      moveFlipper(flipperController.calculate(getFlipperPos(), setPoint));
  }

  

 




  public void stop() {
      lift.set(0);
      flipper.set(0);
  }       
}
