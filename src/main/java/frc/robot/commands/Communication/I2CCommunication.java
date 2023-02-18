// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Communication;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.I2C;
import static frc.robot.Constants.*;

import java.nio.ByteBuffer;

public class I2CCommunication extends CommandBase {
  private static I2C Wire = new I2C(KI2CPort, KI2CAddress);
  /** Creates a new I2CCommunication. */
  public I2CCommunication() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Returns true if success. Can ignore.
  public boolean sendData(KLEDSTATE desiredState)
  {
    int dataToSend = 0;
    boolean isSuccess = true;
    switch (desiredState)
    {
      case OFF:
      dataToSend = 0;
      break;
      case YELLOW:
      dataToSend = 1;
      break;
      case PURPLE:
      dataToSend = 2;
      break;
      default:
      isSuccess = false;
    }
    if (isSuccess)
    {
      isSuccess = Wire.writeBulk(ByteBuffer.allocate(4).putInt(dataToSend).array());
      isSuccess = !isSuccess;
      return isSuccess;
    }
    return false;
  }

  // Function most likely not nessecary, returns 404 if error
  public int readData()
  {
    ByteBuffer readData = ByteBuffer.allocateDirect(4); // past tense, read, not read.
    int finalData = 0;
    boolean isSuccess = Wire.read(KI2CAddress, 4, readData);
    isSuccess = !isSuccess;
    try 
    {
      finalData = readData.getInt();
    }
    catch(Exception e)
    {
      isSuccess = false;
    }
    if (isSuccess == false)
    {
      return 404;
    }
    else
    {
      return finalData;
    }
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