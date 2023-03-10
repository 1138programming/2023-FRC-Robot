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
    private DigitalInput HallEffectSensorIn;
    private DigitalInput HallEffectSensorOut;

    private boolean orientationMode;

    public Orientation() {

        orientationLeftMotor = new CANSparkMax(KOrientationLeftMotorID, MotorType.kBrushless);
        orientationRightMotor = new CANSparkMax(KOrientationRightMotorID, MotorType.kBrushless);

        orientationMotorExtension = new TalonSRX(KOrientationMotorExtensionID);
        
        DoorControl = new DigitalInput (KOrientationkDoorControlID);
        HallEffectSensorIn = new DigitalInput(KOrientationMagSensorInID);
        HallEffectSensorOut = new DigitalInput(KOrientationMagSensorOutID);

        orientationLeftMotor.setIdleMode(IdleMode.kBrake);
        // orientationMotorExtension.setIdleMode(IdleMode.kBrake);
        
        orientationRightMotor.follow(orientationLeftMotor, KOrientationRightMotorReversed);
    }

    /**
     * This command makes both motors move, because although it looks like only one moves, the follow() command is used on the right motor earlier.
     */
    public void moveOrientationLeftandRightMotors() {
        if (orientationMode) {
            orientationLeftMotor.set(KConeLeftandRightMotorSpeeds);
        }
        else if (!orientationMode) {
            orientationLeftMotor.set(KCubeLeftandRightMotorSpeeds);
        }
    }

    /**
     * This command makes both motors move, because although it looks like only one moves, the follow() command is used on the right motor earlier.
     */
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

    public boolean getMagSensorOut() {
        return !HallEffectSensorOut.get();
    }

    public boolean getMagSensorIn(){
        return !HallEffectSensorIn.get();
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putBoolean("Magnetic Limit", getMagSensorOut());
        SmartDashboard.putBoolean("Magnetic Limit!", getMagSensorIn());
    }
}
