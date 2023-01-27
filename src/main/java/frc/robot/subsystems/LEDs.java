// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  AddressableLED ledStrip;
  AddressableLEDBuffer ledBuffer;
  /** Creates a new LEDs. */
  public LEDs() {
    ledStrip = new AddressableLED(KLEDPort);
    ledBuffer = new AddressableLEDBuffer(KLEDBuffer);
  }

  public void ledState(KLEDSTATE state) {
    switch (state){
      case OFF:

      break;

      case YELLOW:
      // --iimlfor(i : ledBuffer.getLengt-)
      ledStrip.rgb(230, 232, 44)

      break;

      case PURPLE:
      break;
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
