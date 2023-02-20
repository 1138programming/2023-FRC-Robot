package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Endgame extends SubsystemBase {

    private Servo endgameLinearServoTop;
    private Servo endgameLinearServoBottom;
    //private DigitalInput endgameBottomIRSensor;
    private final MedianFilter ultrasonicFilter = new MedianFilter(5);
    private double pos;
    private Ultrasonic endgameUltrasonic;
    private double ultrasonicMeasurement;

    private VictorSP vex = new VictorSP(6);
     

    public Endgame() {
        
        //endgameBottomIRSensor = new DigitalInput(KEndgameIRSensor);
        endgameLinearServoTop = new Servo(KLinearServoTop);
        endgameLinearServoBottom = new Servo(KLinearServoBottom);
        endgameUltrasonic = new Ultrasonic(KEndgameUltrasonicPing, KEndgameUltrasonicEcho);

        // Bounds copied from 2022 FRC robot.. the values are most likely wrong. Check with Patrick for values possibly
        endgameLinearServoTop.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
        endgameLinearServoBottom.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
        Ultrasonic.setAutomaticMode(true);
    }
    
    @Override
    public void periodic()
    {
        ultrasonicMeasurement = endgameUltrasonic.getRangeInches();
        ultrasonicMeasurement = ultrasonicFilter.calculate(ultrasonicMeasurement);
        if (ultrasonicMeasurement < 249) {
            SmartDashboard.putNumber("Ultrasonic in.", ultrasonicMeasurement);
        }
        else
        {
            SmartDashboard.putString("Ultrasonic in.", "invalid");
        }
        // endgameUltrasonic.ping();
    }

    public void moveServo(boolean negative) {
        //endgameLinearServoTop.set(pos);
        pos = endgameLinearServoBottom.get();

        SmartDashboard.putNumber("servopos", pos);
        if (negative)
        {
            pos -= 0.02;
        }
        else if (!negative) 
        {
            pos += 0.02;
        }
        if (pos > 1)
        {
            pos = 1;
        }
        if (pos < 0)
        {
            pos = -1;
        }
        endgameLinearServoBottom.setPosition(pos);
    }

    public void moveVex(double speed) {
        vex.set(speed);
        SmartDashboard.putBoolean("pwm", vex.isAlive());
    }
    // public boolean isBottomIRSensorPressed()
    // {
    //     return endgameBottomIRSensor.get();
    // }
}
