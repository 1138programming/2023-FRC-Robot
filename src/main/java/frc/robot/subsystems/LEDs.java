// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  AddressableLED ledStrip;
  AddressableLEDBuffer ledBuffer;

  private boolean intakeMode;

  private boolean defenseLightsOn = false;
  private boolean defenseMode = false;

  private Timer timer;

  public LEDs() {
    ledStrip = new AddressableLED(KLEDPort);
    ledBuffer = new AddressableLEDBuffer(KLEDBuffer);

    ledStrip.setLength(ledBuffer.getLength());
    ledStrip.setData(ledBuffer);
    ledStrip.start();

    timer = new Timer();
  }

  @Override 
  public void periodic() {
    if (defenseMode && timer.get() > 0.5) {
      toggleDefenseLights();
      timer.reset();
    }
  }

  public void ledsOn() {
    ledStrip.start();
  }

  public void ledsOff() {
    ledStrip.stop();
  }

  public void setCubeMode() {
    setLEDs(119, 11, 219);
    intakeMode = KCubeMode;
  }
  
  public void setConeMode() {
    setLEDs(200, 200, 0);
    intakeMode = KConeMode;
  }

  public void toggleDefenseLights() {
    if (defenseLightsOn) {
      stopLEDs();
      defenseLightsOn = false;
    }
    else {
      setDefenseLights();
      defenseLightsOn = true;
    }
  }

  public void endDefenseMode() {
    if (intakeMode == KConeMode) {
      setConeMode();
    }
    else if (intakeMode == KCubeMode) {
      setCubeMode();
    }
    else {
      stopLEDs();
    }
    timer.stop();
  }
  
  //DEFENSE Mode
  public void setDefenseLights() {
    setLEDs(173, 6, 6);
  }

  public void setDefenseMode(boolean mode) {
    this.defenseMode = mode;
    if (!mode) {
      endDefenseMode();
    }
    else {
      setDefenseLights();
      timer.restart();
    }
  }

  public boolean getDefenseMode() {
    return defenseMode;
  }

  public void stopLEDs() {
    setLEDs(0, 0, 0);
  }

  public void setLEDs(int r, int g, int b) {
    for (int i = 0; i < KLEDBuffer; i++) {
      ledBuffer.setRGB(i, r, g, b);
    }
    ledStrip.setData(ledBuffer);
  }
   
  // get the operating mode of the intake
  public boolean isConeMode() {
    return intakeMode; 
  }
}
