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

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*; // Pnuematics


public class Claw extends SubsystemBase {
  

  private Servo claw;
  private Servo wrist;
  private DigitalInput BaseChecker;
  private DigitalInput TipChecker;
  private boolean scoringMode;
  private boolean toRotateWrist;

  private double wristConstant;

  public Claw() 
  {
    claw = new Servo(KClawServo);
    wrist = new Servo (KWristServo);
    BaseChecker = new DigitalInput (KOrientationkBaseCheckerID);
    TipChecker = new DigitalInput (KOrientationkTipCheckerID);
    // claw.setBounds(1.0, 1.8, 1.5, 1.2, 1.0);
    double offset = 0.095;
    wrist.setBounds(2.4 + offset, 1.502, 1.5, 1.498, 0.6 - offset);
    claw.set(KOpenClaw);
    wrist. set(wristConstant);

    wristConstant = 0; 

    scoringMode = false;

    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ClawPos", claw.getPosition());
    SmartDashboard.putNumber("WristPos", wristConstant);
  }

  public void moveClaw(double setpoint) {
      // setpoint *= 1/135; // converts from degrees (135 max)
      claw.set(setpoint);

  }
  public void moveWrist (double setPoint) {
       wrist.set(setPoint);
  }
  public void moveWristIncrement() {
      wristConstant += .05; 
      if (wristConstant > 1) {
        wristConstant = 1;
      }
      
      wrist.set(wristConstant);
  } 
  public void moveWristDecrement() {
     wristConstant -= .05; 
     if (wristConstant < 0) {
       wristConstant = 0;
    }
    
    wrist.set(wristConstant);
} 



  public void closeClaw() {
      if (scoringMode) {
          claw.set(KCloseClawCone);  
      }
      else {
          claw.set(KCloseClawCube);
      }
  }

  public void openClaw() {
    
    claw.set(KOpenClaw);

  }

  public void setConeMode() {
      scoringMode = KConeMode; 
  }

  public boolean isConeMode() {
      return scoringMode;
  }

  public boolean getBaseSensor() {
      return BaseChecker.get();
  }

  public boolean getTipSensor() {
      return TipChecker.get();
  }
  public void setCubeMode() {
    scoringMode = KCubeMode;
  }
  public void updateScoringWristStatus() {
    if (scoringMode) {
        if (getTipSensor()) {
          toRotateWrist = false;
        }
        else  if (getBaseSensor()) {
            toRotateWrist = true;
        }
    }
    else {
        toRotateWrist = false;
    }
  }
  public boolean getWristStatus() {
    return toRotateWrist;
  }
  
  public void stop() {
      claw.set(0);
      wrist.set(0);
  }       
}
