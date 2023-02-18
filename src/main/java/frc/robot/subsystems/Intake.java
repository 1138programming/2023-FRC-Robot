// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private TalonSRX flex;
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
    flex = new TalonSRX(KFlexIntakeId);

    intakeController = new PIDController(KIntakeP, KIntakeI, KIntakeD);

    intakeLimit = new DigitalInput(KIntakeLimitId);

    ledStrip = new AddressableLED(KLEDPort);
    ledBuffer = new AddressableLEDBuffer(KLEDBuffer);

    ledStrip.setLength(ledBuffer.getLength());
    ledStrip.setData(ledBuffer);
    ledStrip.start();

  }

  public void spaghettiSpin() {
    if (intakeMode) {
      spaghetti.set(ControlMode.PercentOutput, KIntakeConeSpaghettitSpeed);
    }
    else if (!intakeMode) {
      spaghetti.set(ControlMode.PercentOutput, KIntakeCubeSpaghettitSpeed);
    }
    
  }

  public void ledsOff() {
    ledStrip.stop();
  }

  public void setCubeMode() {
    
    intakeMode = false;

    // set the led strip to purple
    ledStrip.start();
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 119, 11, 219);
    }
    ledStrip.setData(ledBuffer);
  }

  public void setConeMode() {
    intakeMode = true; 

    ledStrip.start();
        for (int i = 0; i < ledBuffer.getLength(); i++) {
          ledBuffer.setRGB(i, 230, 232, 44);
        }
        ledStrip.setData(ledBuffer);
  }
  
  // get the operating mode of the intake
  public boolean getMode() {
    return intakeMode; 
  }

  public void swivelSpinToPos(double setPoint) {
    moveSwivel(intakeController.calculate(getIntakeEncoder(), setPoint));
  }

  public void moveSwivel(double speed) {
    if (!getIntakeLimitSwitch()) {
      swivel.set(ControlMode.PercentOutput, speed);
    } else {
      swivel.set(ControlMode.PercentOutput, 0);
    }
  }



  public void flexSpin() {
    if (intakeMode) {
      flex.set(ControlMode.PercentOutput, KIntakeConeFlexSpeed);
    }
    else if(!intakeMode) {
      flex.set(ControlMode.PercentOutput, KIntakeCubeFlexSpeed);
    }
  }



  public double getIntakeEncoder() {
    return swivel.getSelectedSensorPosition();
  }


  public boolean getIntakeLimitSwitch() {
    return intakeLimit.get();
  } 
  
  public void intakeStop() {
    flex.set(ControlMode.PercentOutput, 0);
    swivel.set(ControlMode.PercentOutput, 0);
    spaghetti.set(ControlMode.PercentOutput, 0);
  }
}
