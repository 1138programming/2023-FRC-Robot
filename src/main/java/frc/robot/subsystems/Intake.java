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

  private DigitalInput intakeTopLimit;
  private DigitalInput intakeBottomLimit;

  AddressableLED ledStrip1;
  AddressableLED ledStrip2;
  AddressableLEDBuffer ledBuffer;

  private boolean intakeMode;
  private boolean defenseMode = false;

  public Intake() {
    spaghetti = new TalonSRX(KSpaghettiIntakeId);
    swivel = new TalonSRX(KSwivelIntakeId);
    
    spaghetti.setInverted(true); 

    intakeController = new PIDController(KIntakeP, KIntakeI, KIntakeD);

    intakeBottomLimit = new DigitalInput(KIntakeTopLimitId);
    intakeTopLimit = new DigitalInput(KIntakeBottomLimitId);

    ledStrip1 = new AddressableLED(KLEDPort1);
    ledStrip2 = new AddressableLED(KLEDPort2);
    ledBuffer = new AddressableLEDBuffer(KLEDBuffer);

    ledStrip1.setLength(ledBuffer.getLength());
    ledStrip1.setData(ledBuffer);
    ledStrip1.start();
    ledStrip2.setLength(ledBuffer.getLength());
    ledStrip2.setData(ledBuffer);
    ledStrip2.start();
  }

  @Override 
  public void periodic() {
    SmartDashboard.putBoolean("Mode", intakeMode);
    SmartDashboard.putNumber("Intake Encoder", getIntakeEncoder());
    if (getTopLimitSwitch()) {
      setIntakeEncoder(0);
    }
    if (getBottomLimitSwitch()) {
      setIntakeEncoder(KSwivelBottomPosition);
    }
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
  
  public void spaghettiSpinReverse() {
    if (intakeMode) {
      spaghetti.set(ControlMode.PercentOutput, -KIntakeConeSpaghettitSpeed);
    }
    else if (!intakeMode) {
      spaghetti.set(ControlMode.PercentOutput, -KIntakeCubeSpaghettitSpeed);
    }
  }

public void setLEDToColor(int R, int G, int B) {
  ledStrip1.start();
  ledStrip2.start();
  for (int i = 0; i < ledBuffer.getLength(); i++) {
    ledBuffer.setRGB(i, R, G, B);
  }
  ledStrip1.setData(ledBuffer);
  ledStrip2.setData(ledBuffer);
}



  public void ledsOff() {
    ledStrip1.stop();
    ledStrip2.stop();
  }
  
  public void setCubeMode() {
    intakeMode = KCubeMode;
    defenseMode = false; 

    // set the led strip to purple
    setLEDToColor(119, 11, 219);
  }

  public void setConeMode() {
    intakeMode = KConeMode;
    defenseMode = false; 
    
    // yellow
    ledStrip1.start();
    ledStrip2.start();
        for (int i = 0; i < ledBuffer.getLength(); i++) {
          ledBuffer.setRGB(i, 150, 150, 0);
        }
        ledStrip1.setData(ledBuffer);
        ledStrip2.setData(ledBuffer);
  }

  public void toggleDefenseMode() {
    if (!defenseMode) {
      defenseMode = true;
      ledStrip1.start();
      ledStrip2.start();
          for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 200, 0, 0);
          }
          ledStrip1.setData(ledBuffer);
          ledStrip2.setData(ledBuffer);
    }
    else {
      defenseMode = false;
      if (intakeMode == KConeMode) {
        setConeMode();
      }
      else {
        setCubeMode();
      }
    }
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
    if (speed > 0 && !getTopLimitSwitch()) {
      swivel.set(ControlMode.PercentOutput, speed);
    }
    else if (speed < 0 && !getBottomLimitSwitch()) {
      swivel.set(ControlMode.PercentOutput, -speed);
    }
    else {
      swivel.set(ControlMode.PercentOutput, 0);
    }
  }

  public void setIntakeEncoder(double position) {
    swivel.setSelectedSensorPosition(position);
  }
  public double getIntakeEncoder() {
    return swivel.getSelectedSensorPosition();
  }

  public boolean getTopLimitSwitch() {
    return intakeTopLimit.get();
  } 
  public boolean getBottomLimitSwitch() {
    return intakeBottomLimit.get();
  } 
  
  public void intakeStop() {
    swivel.set(ControlMode.PercentOutput, 0);
    spaghetti.set(ControlMode.PercentOutput, 0);
  }
}
