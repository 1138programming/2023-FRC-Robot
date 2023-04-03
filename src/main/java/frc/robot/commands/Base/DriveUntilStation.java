// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Base;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Base;
import static frc.robot.Constants.*;

public class DriveUntilStation extends CommandBase {
  private double lastPitch;
  private double pitchChange = 0;
  private Base base;
  
  private PIDController balanceController;
  private PIDController pitchChangeController;

  private boolean onStation = false;
  private double offStationSpeed = 0.75;

  /** Creates a new AutoBalance. */
  public DriveUntilStation(Base base) {
    this.base = base;
    balanceController = new PIDController(KBalanceP, KBalanceI, KBalanceD);
    pitchChangeController = new PIDController(0.1, 0, 0);
    // balanceController.
    addRequirements(base);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // base.resetAllRelEncoders();
    onStation = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    base.drive(0.2, 0, 0, true, KPhysicalMaxDriveSpeedMPS);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(base.getPitch()) > 12;
  }
}
