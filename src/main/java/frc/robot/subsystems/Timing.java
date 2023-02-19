// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static frc.robot.Constants.*;



public class Timing extends TimedRobot {
  private Ultrasonic endgameUltrasonic;
  private double ultrasonicMeasurement;

  @Override
  public void robotInit() {
    endgameUltrasonic = new Ultrasonic(KEndgameUltrasonicPing, KEndgameUltrasonicEcho);
    addPeriodic(() -> {
      ultrasonicMeasurement = endgameUltrasonic.getRangeInches();
    SmartDashboard.putNumber("In. Measurement", ultrasonicMeasurement);
    System.out.println(ultrasonicMeasurement);
  }, 0.1);
      super.robotInit();
  }
}
