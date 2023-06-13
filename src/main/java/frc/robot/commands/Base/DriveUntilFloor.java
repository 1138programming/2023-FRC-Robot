// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Base;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Base;
import static frc.robot.Constants.*;

public class DriveUntilFloor extends CommandBase {
  private Base base;

  private int stage = 1;
  private double overStationSpeed = 0.3;

  /** Creates a new AutoBalance. */
  public DriveUntilFloor(Base base) {
    this.base = base;
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
    
    base.drive(overStationSpeed, 0, 0, true, KPhysicalMaxDriveSpeedMPS);
    if (base.getPitch() < -3) {
      stage = 2;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(base.getPitch()) < 3 && stage == 2;
  }
}
