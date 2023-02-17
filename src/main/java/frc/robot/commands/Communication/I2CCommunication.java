// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Communication;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.I2C;
import static frc.robot.Constants.*;

public class I2CCommunication extends CommandBase {
  private static I2C Wire = new I2C(KI2CPort, 4);
  /** Creates a new I2CCommunication. */
  public I2CCommunication() {
    // Use addRequirements() here to declare subsystem dependencies.
  }
  public void sendData()
  {
    Wire.write(KBackLeftDriveID, sizeof())
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
