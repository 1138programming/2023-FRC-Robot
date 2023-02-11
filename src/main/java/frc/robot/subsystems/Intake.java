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

public class Intake extends SubsystemBase {
  private TalonSRX flex;
  private TalonSRX swivel;
  private TalonSRX spaghetti;

  private PIDController intakeController;

  private DigitalInput intakeLimit;

  public Intake() {
    spaghetti = new TalonSRX(KSpaghettiIntakeId);
    swivel = new TalonSRX(KKSwivelIntakeId);
    flex = new TalonSRX(KFlewIntakeId);

    intakeController = new PIDController(KIntakeP, KIntakeI, KIntakeD);

    intakeLimit = new DigitalInput(KIntakeLimitId);
  }

  public void spaghettiSpin(double speed) {
    spaghetti.set(ControlMode.PercentOutput, speed);
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



  public void flexSpin(double speed) {
    flex.set(ControlMode.PercentOutput, speed);

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
