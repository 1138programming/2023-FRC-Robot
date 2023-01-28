// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private TalonSRX flex; 
  private TalonSRX swivel;
  private TalonSRX spaghetti;
  private PIDController intakeController;
  public Intake() {
    spaghetti = new TalonSRX(KSpaghettiIntakeId);
    swivel = new TalonSRX(KLeftIntakeId);
    flex = new TalonSRX(KRightIntakeId);
    intakeController = new PIDController(KIntakeP,KIntakeI, KIntakeD);

  }
    
  public void spaghettiSpin (double speed){
    spaghetti.set(ControlMode.PercentOutput, speed);
  }
  public void spaghettiStop () {
    spaghetti.set(ControlMode.PercentOutput, 0);
  }
  public void swivelSpinToPos (double setPoint ) {
    swivel.set(ControlMode.PercentOutput, intakeController.calculate(getIntakeEncoderRaw(), setPoint));
  
  }
  public void swivelStop () {
    swivel.set(ControlMode.PercentOutput, 0);
  }

  public void flexspin (double speed) {
    flex.set(ControlMode.PercentOutput, speed);
    
  }
  public void flewStop () {
    flex.set(ControlMode.PercentOutput, 0);
  } 
  public double getEncoder () {
    return swivel.getSelectedSensorPosition();
  }
 
  public double getIntakeEncoderRaw() {
    return swivel.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
