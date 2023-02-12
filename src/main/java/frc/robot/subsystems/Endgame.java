package frc.robot.subsystems;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Endgame extends SubsystemBase {

    private Servo endgameLinearServoTop;
    private Servo endgameLinearServoBottom;
    //private DigitalInput endgameBottomIRSensor;
    private final MedianFilter ultrasonicFilter = new MedianFilter(5);
    private Ultrasonic endgameUltrasonic;
    private double ultrasonicMeasurement;

    public Endgame() {
        //endgameBottomIRSensor = new DigitalInput(KEndgameIRSensor);
        endgameLinearServoTop = new Servo(KLinearServoTop);
        endgameLinearServoBottom = new Servo(KLinearServoBottom);
        endgameUltrasonic = new Ultrasonic(KEndgameUltrasonicPing, KEndgameUltrasonicEcho);

        // Bounds copied from 2022 FRC robot.. the values are most likely wrong. Check with Patrick for values possibly
        endgameLinearServoTop.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
        endgameLinearServoBottom.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
    }

    @Override
    public void periodic()
    {
        ultrasonicMeasurement = endgameUltrasonic.getRangeMM();
        SmartDashboard.putNumber("Ultrasonic in.", ultrasonicMeasurement);
        System.out.println(ultrasonicMeasurement);
    }

    public void moveServo(double pos) {
        endgameLinearServoTop.set(pos);
        endgameLinearServoBottom.set(pos);
    }
    // public boolean isBottomIRSensorPressed()
    // {
    //     return endgameBottomIRSensor.get();
    // }
}
