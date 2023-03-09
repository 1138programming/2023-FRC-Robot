// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Base;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Base;
import static frc.robot.Constants.*;

public class AutoBalance extends CommandBase {
  private Base base;
  private PIDController balanceController;
  private boolean onStation = false;

  /** Creates a new AutoBalance. */
  public AutoBalance(Base base) {
    this.base = base;
    balanceController = new PIDController(KBalanceP, KBalanceI, KBalanceD);
    addRequirements(base);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (onStation == false) {
      base.drive(0, 0.3, 0, false, KPhysicalMaxDriveSpeedMPS);
      if (base.getRoll() > 20) {
        onStation = true;
      }
    }
    else {
      if (Math.abs(base.getRoll()) < 6) {
        base.lockWheels();
      }
      double speed = balanceController.calculate(base.getRoll(), 0);
      base.drive(0, speed, 0, false, KPhysicalMaxDriveSpeedMPS);
    }
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
