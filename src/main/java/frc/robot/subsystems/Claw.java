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

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*; // Pnuematics


public class Claw extends SubsystemBase {
  

  private Servo claw;
  private Servo wrist;
  private DigitalInput BaseChecker;
  private DigitalInput TipChecker;
  private boolean scoringMode;
  private boolean toRotateWrist;

  public Claw() 
  {
    claw = new Servo(KClawServo);
    wrist = new Servo (KWristServo);
    BaseChecker = new DigitalInput (KOrientationkBaseCheckerID);
    TipChecker = new DigitalInput (KOrientationkTipCheckerID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void moveClaw(double setpoint) {
      setpoint *= 1/135; // converts from degrees (135 max)
      claw.set(setpoint);
  }
  public void moveWrist(double setpoint) {
      setpoint *= 1/300;
      wrist.set(setpoint);
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
      if (scoringMode) {
          claw.set(KOpenClaw);
      }
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
