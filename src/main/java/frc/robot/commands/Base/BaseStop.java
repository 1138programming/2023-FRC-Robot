// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Base;

import frc.robot.Robot;
import frc.robot.subsystems.Base;
import static frc.robot.Constants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class BaseStop extends CommandBase {
  private final Base base;

  /** Creates a new DriveWithJoysticks. */
  public BaseStop(Base base) {
    this.base = base;
    addRequirements(base);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    base.resetAllRelEncoders();  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    base.drive(0, 0, 0, true, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }


}