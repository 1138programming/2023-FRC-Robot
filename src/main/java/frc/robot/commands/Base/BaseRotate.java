// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Base;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Base;
import static frc.robot.Constants.*;

public class BaseRotate extends CommandBase {
  private Base base;
  private double angle = 0;

  private double rotP = 4.5;
  private double rotI = 0;
  private double rotD = 0;

  private PIDController rotController = new PIDController(rotP, rotI, rotD);
  /** Creates a new BaseRotate. */
  public BaseRotate(Base base) {
    this.base = base;
    addRequirements(base);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angle = base.getHeadingDeg();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    angle = base.getHeadingDeg();
    double rot = rotController.calculate(angle, 180);
    base.drive(0, 0, 0.5, true, KPhysicalMaxDriveSpeedMPS);
    base.drive(0, 0, rot, true, KPhysicalMaxDriveSpeedMPS);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    base.drive(0, 0, 0, true, KPhysicalMaxDriveSpeedMPS);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return false;
    return Math.abs(180 - angle) < 5;
  }
}
