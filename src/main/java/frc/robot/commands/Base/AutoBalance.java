// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Base;

import static frc.robot.Constants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Base;

public class AutoBalance extends CommandBase {
  private final Base base;

  private PIDController rotController;
  private double rotationP = KBalanceP;
  private double rotationI = KBalanceI;
  private double rotationD = KBalanceD;

  private int counter = 0;
  private int counter2 = 0;
  private double currentRot;
  private int multiplier = 1;

  /** Creates a new AutoBalance. */
  public AutoBalance(Base base) {
    this.base = base;
    rotController = new PIDController(rotationP, rotationI, rotationD);

    addRequirements(base);

  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (base.getHeadingDeg() < 0) {
      multiplier = 1;
    } else {
      multiplier = -1;
    }
    double xSpeed = 0;
    currentRot = base.getPitch();
    if (Math.abs(currentRot) > 2) {
      counter = 0;
      xSpeed = rotController.calculate(currentRot, 0.69) * multiplier;
    } else {
      counter++;
    }

    if (counter2 <= 30) {
      base.drive(xSpeed, 0, 0, true, KPhysicalMaxDriveSpeedMPS);
      counter2++;
    } else {
      counter2++;
      if (counter2 >= 35) {
        counter2 = 0;
      }
    }
    SmartDashboard.putNumber("counter", counter2);
  }

  @Override
  public void end(boolean interrupted) {
    base.stop();
  }

  @Override
  public boolean isFinished() {
    return counter > 100;
  }
}
