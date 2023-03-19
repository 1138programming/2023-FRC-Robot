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
  private Base base;
  private PIDController balanceController;
  private boolean onStation = false;
  private double onStationSpeed = 0.15;
  private double offStationSpeed = 0.15;

  /** Creates a new AutoBalance. */
  public AutoBalance(Base base) {
    this.base = base;
    balanceController = new PIDController(KBalanceP, KBalanceI, KBalanceD);
    addRequirements(base);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // base.resetAllRelEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("roll", base.getRoll());
    SmartDashboard.putNumber("pitch", base.getPitch());
    // SmartDashboard.putNumber("pitch", base.());
    double roll = base.getPitch();
    if (onStation == false) {
      base.drive(offStationSpeed, 0, 0, false, KPhysicalMaxDriveSpeedMPS);
      if (Math.abs(roll) > 10) {
        onStation = true;
      }
    }
    else {
      if (Math.abs(roll) < 5) {
        base.lockWheels();
        // base.drive(0, 0, 0, false, KPhysicalMaxDriveSpeedMPS);
      }
      else {

        double speed = balanceController.calculate(roll, 0);
        base.drive(-speed, 0, 0, false, KPhysicalMaxDriveSpeedMPS);
        // if (roll < 0) {
        //   base.drive(-onStationSpeed, 0, 0, false, KPhysicalMaxDriveSpeedMPS);
        // }
        // else {
        //   base.drive(onStationSpeed, 0, 0, false, KPhysicalMaxDriveSpeedMPS);
        }
      }
    }
  // }

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
