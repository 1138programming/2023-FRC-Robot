package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax; // Neos and 775
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.CANSparkMaxLowLevel.MotorType; // Covers Neos and 775 
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class Claw extends SubsystemBase {
  

  private CANSparkMax roller;
  
  private boolean scoringMode;

  public Claw() 
  {
    roller = new CANSparkMax(KClawMotorID, MotorType.kBrushless);

    //scoringMode = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
  // spin roller
  public void spinRoller(double speed) {
    roller.set(KClawMotorSpeed);
  }


 
}
