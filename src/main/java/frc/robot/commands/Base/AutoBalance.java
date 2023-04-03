// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Base;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Base;
import static frc.robot.Constants.*;

public class AutoBalance extends CommandBase {
  private double lastPitch;
  private double pitchChange = 0;
  private Base base;
  
  private PIDController balanceController;
  private PIDController pitchChangeController;

  private boolean onStation = false;
  private double onStationSpeed = 0.1;
  private double offStationSpeed = 0.2;

  /** Creates a new AutoBalance. */
  public AutoBalance(Base base) {
    this.base = base;
    balanceController = new PIDController(KBalanceP, KBalanceI, KBalanceD);
    pitchChangeController = new PIDController(0.1, 0, 0);
    // balanceController.
    addRequirements(base);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    onStation = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("pitch", base.getPitch());
    double pitch = base.getPitch();
    pitchChange = (pitch - lastPitch);
    lastPitch = pitch;

    SmartDashboard.putNumber("pitch change", pitchChange);

    double speed = balanceController.calculate(pitch, 0);
    if (Math.abs(pitchChange) > 0.3 || Math.abs(pitch) < 3) {
      speed = 0;
    }
    base.drive(-speed, 0, 0, false, KPhysicalMaxDriveSpeedMPS);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
