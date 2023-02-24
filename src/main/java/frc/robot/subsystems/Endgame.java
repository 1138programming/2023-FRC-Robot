package frc.robot.subsystems;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Endgame extends SubsystemBase {

    private Servo endgameLinearServoFront;
    private Servo endgameLinearServoBack;
    private DigitalInput endgameLimitSwitch;

    private double newPos;

    public Endgame() {
       
        endgameLinearServoFront = new Servo(KLinearServoFront);
        endgameLinearServoBack = new Servo(KLinearServoBack);
         
        
        // Bounds copied from 2022 FRC robot.. the values are most likely wrong. Check with Patrick for values possibly
        endgameLinearServoFront.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
        endgameLinearServoBack.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
        SmartDashboard.putNumber("servopos", 0.5);
    }
    
    @Override
    public void periodic() {
        // SmartDashboard.putNumber("servopos", newPos);
        newPos = SmartDashboard.getNumber("servopos", 0);
        // newPos = endgameLinearServoBottom.get();
        
        SmartDashboard.putNumber("actual servo pos", endgameLinearServoFront.get());
        // SmartDashboard.putNumber("", KAngleD)
        SmartDashboard.putBoolean("limitswitch", isLimitSwitchPressed());
    }
    
    public void moveServo(double pos) {
        endgameLinearServoFront.set(pos);
        endgameLinearServoBack.set(pos);
    }

    public void moveServoShuffleboard() {
        endgameLinearServoFront.set(newPos);
        endgameLinearServoBack.set(newPos);
    }

    public boolean isLimitSwitchPressed() {
        return endgameLimitSwitch.get();
    }
}
