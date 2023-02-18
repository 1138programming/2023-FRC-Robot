package frc.robot.subsystems;

import static frc.robot.Constants.*;
import com.revrobotics.CANSparkMax; // Covers Neo's
import com.revrobotics.CANSparkMaxLowLevel.MotorType; // Cover's Neo's
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput; // Proximity Sensors
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 

/* 
 * 3 motors: Undecided: Neos or 775's
 * 3 sensors most likely
 */
public class Orientation extends SubsystemBase {
    private CANSparkMax orientationLeftMotor;
    private CANSparkMax orientationRightMotor;
    private CANSparkMax orientationMotorExtension;
    
    private DigitalInput Sensor1;
    private DigitalInput Sensor2;
    private DigitalInput Sensor3;
    private DigitalInput HallEffectSensor1;
    private DigitalInput HallEffectSensor2;
    
    
    public Orientation() {
        orientationLeftMotor = new CANSparkMax(KOrientationLeftMotorID, MotorType.kBrushless);
        orientationRightMotor = new CANSparkMax(KOrientationRightMotorID, MotorType.kBrushless);
        orientationMotorExtension = new CANSparkMax(KOrientationMotorExtensionID, MotorType.kBrushless);
        Sensor1 = new DigitalInput (KOrientationSensor1ID);
        Sensor2 = new DigitalInput (KOrientationSensor2ID);
        Sensor3 = new DigitalInput (KOrientationSensor3ID);
        HallEffectSensor1 = new DigitalInput(KOrientationHallEffectSensor1ID);
        HallEffectSensor2 = new DigitalInput(KOrientationHallEffectSensor2ID);

        orientationRightMotor.follow(orientationLeftMotor);
    }

    public void moveOrientationLeftandRightMotors(double speed) {
        orientationLeftMotor.set(speed);
    }

    public void moveOrientationMotorExtension(double speed) {
        orientationMotorExtension.set(speed);
    }

    public void stopOrientationLeftandRightMotors() {
        orientationLeftMotor.set(0);
    }

    public void stopOrientationMotorExtension() {
        orientationMotorExtension.set(0);
    }

    public Boolean getSensor1() {
        return Sensor1.get();
    }

    public Boolean getSensor2() {
        return Sensor2.get();
    }

    public Boolean getSensor3() {
        return Sensor3.get();
    }

}
