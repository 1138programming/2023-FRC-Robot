package frc.robot.subsystems;

import static frc.robot.Constants.*;
import com.revrobotics.CANSparkMax; // Covers Neo's
import com.revrobotics.CANSparkMaxLowLevel.MotorType; // Cover's Neo's
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput; // Sensors

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 
import com.revrobotics.CANSparkMax.IdleMode;

/* 
 * 3 motors - 2 Neos or 775s & 1 Snow Blower - most likely TalonSRX, but can be VictorSPX or Jaguar (hopefully not that)
 * 1 encoder for extension
 * 3 sensors most likely
 */
public class Orientation extends SubsystemBase {
    private CANSparkMax orientationLeftMotor;
    private CANSparkMax orientationRightMotor;
    private TalonSRX orientationMotorExtension;
    
    private DigitalInput DoorControl;
    private DigitalInput HallEffectSensor1;
    private DigitalInput HallEffectSensor2;

    private boolean orientationMode;

    public Orientation() {

        orientationLeftMotor = new CANSparkMax(KOrientationLeftMotorID, MotorType.kBrushless);
        orientationRightMotor = new CANSparkMax(KOrientationRightMotorID, MotorType.kBrushless);

        orientationMotorExtension = new TalonSRX(KOrientationMotorExtensionID);
        
        DoorControl = new DigitalInput (KOrientationkDoorControlID);
        HallEffectSensor1 = new DigitalInput(KOrientationHallEffectSensor1ID);
        HallEffectSensor2 = new DigitalInput(KOrientationHallEffectSensor2ID);

        orientationLeftMotor.setIdleMode(IdleMode.kBrake);
        // orientationMotorExtension.setIdleMode(IdleMode.kBrake);
        
        orientationRightMotor.follow(orientationLeftMotor, KOrientationRightMotorReversed);
    }

    public void moveOrientationLeftandRightMotors() {
        if (orientationMode) {
            orientationLeftMotor.set(KConeLeftandRightMotorSpeeds);
        }
        else if (!orientationMode) {
            orientationLeftMotor.set(KCubeLeftandRightMotorSpeeds);
        }
    }

    public void moveOrientationLeftandRightMotors(double speed) {
        orientationLeftMotor.set(speed);
    }

    public void moveOrientationMotorExtension(double speed) {
        orientationMotorExtension.set(ControlMode.PercentOutput, speed);
    }

    public void setCubeMode() {
        orientationMode = KCubeMode;
    }

    public void setConeMode() {
        orientationMode = KConeMode; 
    }

    public boolean isConeMode() {
        return orientationMode;
    }

    public void stopOrientationLeftandRightMotors() {
        orientationLeftMotor.set(0);
    }

    public void stopOrientationMotorExtension() {
        orientationMotorExtension.set(ControlMode.PercentOutput, 0);
    }

    public boolean getDoorSensor() {
        return DoorControl.get();
    }

    public boolean getHallEffectSensor1() {
        return !HallEffectSensor1.get();
    }

    public boolean getHallEffectSensor2(){
        return !HallEffectSensor2.get();
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putBoolean("Magnetic Limit", getHallEffectSensor1());
        SmartDashboard.putBoolean("Magnetic Limit!", getHallEffectSensor2());
    }
}
