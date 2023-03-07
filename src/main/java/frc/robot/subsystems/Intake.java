// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private TalonSRX swivel;
  private TalonSRX spaghetti;

  private PIDController intakeController;

  private DigitalInput intakeLimit;

  AddressableLED ledStrip;
  AddressableLEDBuffer ledBuffer;

  private boolean intakeMode;

  public Intake() {
    spaghetti = new TalonSRX(KSpaghettiIntakeId);
    swivel = new TalonSRX(KSwivelIntakeId);
    
    spaghetti.setInverted(true); 

    intakeController = new PIDController(KIntakeP, KIntakeI, KIntakeD);

    intakeLimit = new DigitalInput(KIntakeLimitId);

    ledStrip = new AddressableLED(KLEDPort);
    ledBuffer = new AddressableLEDBuffer(KLEDBuffer);

    ledStrip.setLength(ledBuffer.getLength());
    ledStrip.setData(ledBuffer);
    ledStrip.start();

  }

  @Override 
  public void periodic() {
    SmartDashboard.putBoolean("Mode", intakeMode);
  }

  /**
   * Spins the "spaghetti" motors (the spinners in the intake)
   */
  public void spaghettiSpin() {
    if (intakeMode) {
      spaghetti.set(ControlMode.PercentOutput, KIntakeConeSpaghettitSpeed);
    }
    else if (!intakeMode) {
      spaghetti.set(ControlMode.PercentOutput, KIntakeCubeSpaghettitSpeed);
    }
  }
  
  /**
   * Spins the motors in reverse, AKA outtaking them. Gage needs to name his functions better.
   */
  public void spaghettiSpinOut() {
    if (intakeMode) {
      spaghetti.set(ControlMode.PercentOutput, -KIntakeConeSpaghettitSpeed);
    }
    else if (!intakeMode) {
      spaghetti.set(ControlMode.PercentOutput, -KIntakeCubeSpaghettitSpeed);
    }
    
  }

public void setLEDToColor(int R, int G, int B) {
  ledStrip.start();
  for (int i = 0; i < ledBuffer.getLength(); i++) {
    ledBuffer.setRGB(i, R, G, B);
  }
  ledStrip.setData(ledBuffer);
}



  public void ledsOff() {
    ledStrip.stop();
  }

  public void setCubeMode() {
    intakeMode = false;

    // set the led strip to purple
    setLEDToColor(119, 11, 219);
  }

  public void setConeMode() {
    intakeMode = true; 
    
    // set LED strip to dark-ish yellow, as when it is
    setLEDToColor(200, 200, 5);
  }
  
  // Possible mode
public void setDefenseMode() {
  setLEDToColor(255, 0, 0);
}

  // get the operating mode of the intake
  public boolean isConeMode() {
    return intakeMode; 
  }

  public void swivelSpinToPos(double setPoint) {
    moveSwivel(intakeController.calculate(getIntakeEncoder(), setPoint));
  }


  /**
   * Moves the swivel at a certain speed
   * @param speed
   */
  public void moveSwivel(double speed) {
    if (!getIntakeLimitSwitch()) {
      swivel.set(ControlMode.PercentOutput, speed);
    } else {
      swivel.set(ControlMode.PercentOutput, 0);
    }
  }


  public double getIntakeEncoder() {
    return swivel.getSelectedSensorPosition();
  }

  public boolean getIntakeLimitSwitch() {
    return intakeLimit.get();
  } 
  
  public void intakeStop() {
    swivel.set(ControlMode.PercentOutput, 0);
    spaghetti.set(ControlMode.PercentOutput, 0);
  }
}
