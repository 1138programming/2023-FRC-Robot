package frc.robot.subsystems;

import static frc.robot.Constants.*;
import com.revrobotics.CANSparkMax; // Covers Neo's
import com.revrobotics.CANSparkMaxLowLevel.MotorType; // Cover's Neo's
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput; // Proximity Sensors
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 

/* 
 * 3 motors: Undecided: Neos or 775's
 * Possibly 10 or more sensors proximity sensor
 */
public class Orientation extends SubsystemBase {
    private CANSparkMax orientationMotor1;
    private CANSparkMax orientationMotor2;
    private CANSparkMax orientationMotorExtension;
    
    private DigitalInput proximitySensor;
    
    // private DoubleSolenoid possiblePiston;
    
    public Orientation() {
        orientationMotor1 = new CANSparkMax(KOrientationMotor1ID, MotorType.kBrushless);
        orientationMotor2 = new CANSparkMax(KOrientationMotor2ID, MotorType.kBrushless);
        orientationMotorExtension = new CANSparkMax(KOrientationMotorExtensionID, MotorType.kBrushless);
        proximitySensor = new DigitalInput (KOrientationProximityID);
        
    }

    public void moveOrientationMotor1(double speed) {
        orientationMotor1.set(speed);
    }

    public void moveOrientationMotor2(double speed) {
        orientationMotor2.set(speed);
    }

    public void moveOrientationMotorExtension(double speed) {
        orientationMotorExtension.set(speed);
    }

    public void stopOrientationMotor1() {
        orientationMotor1.set(0);
    }

    public void stopOrientationMotor2() {
        orientationMotor2.set(0);
    }

    public void stopOrientationMotorExtension() {
        orientationMotorExtension.set(0);
    }

    public Boolean getProximitySensor() {
        return proximitySensor.get();
    }

}
