package frc.robot.subsystems;

import static frc.robot.Constants.*;
import com.revrobotics.CANSparkMax; // Covers Neo's
import com.revrobotics.CANSparkMaxLowLevel.MotorType; // Cover's Neo's
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid; // Pnuematics
import edu.wpi.first.wpilibj.PneumaticsModuleType; // Pnuematics
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*; // Pnuematics
import edu.wpi.first.wpilibj.DigitalInput; // Color Sensor
import edu.wpi.first.wpilibj.util.Color; // Color Sensor
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 
import edu.wpi.first.wpilibj.I2C; // Color Sensor

/* 
 * 2 motors: Undecided: Neos or 775's
 * 1 color sensor
 * 1 proximity sensor
 * Potientially 1 pnuematic piston (Double acting solinoid)
 */
public class Orientation extends SubsystemBase {
    
    private DigitalInput proximitySensor;
    private CANSparkMax orientationMotor1;
    private CANSparkMax orientationMotor2;
    private DoubleSolenoid possiblePiston;
    //private DigitalInput colorSensor; //Assuming it's the Color Sensor V3

    
    public Orientation() {
        proximitySensor = new DigitalInput (KOrientationProximityID);
        orientationMotor1 = new CANSparkMax(KOrientationMotor1ID, MotorType.kBrushless);
        orientationMotor2 = new CANSparkMax(KOrientationMotor2ID, MotorType.kBrushless);
        possiblePiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, KBackLeftDriveID, KBackLeftAngleID);
        
    }

    public void moveOrientationMotor1(double speed) {
        orientationMotor1.set(speed);
    }

    public void moveOrientationMotor2(double speed) {
        orientationMotor2.set(speed);
    }

    public void stopOrientationMotor1(double stop) {
        orientationMotor1.set(0);
    }

    public void stopOrientationMotor2(double stop) {
        orientationMotor2.set(0);
    }

    public Boolean getProximitySensor() {
        return proximitySensor.get();
    }

    public void setOff () {
       possiblePiston.set(kOff);
      }
    
    public void setReverse() {
        possiblePiston.set(kReverse);
    } 

    public void setFoward() {
        possiblePiston.set(kForward);
    }
    /* 
    public Boolean getColorSensor() {
        return colorSensor.get();
    } */

}
