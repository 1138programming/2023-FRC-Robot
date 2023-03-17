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
    private CANSparkMax leftSpin;
    private CANSparkMax rightSpin;
    private TalonSRX extension;
    
    private DigitalInput DoorControl;
    private DigitalInput HallEffectSensorIn;
    private DigitalInput HallEffectSensorOut;

    private boolean orientationMode;

    public Orientation() {

        leftSpin = new CANSparkMax(KOrientationLeftMotorID, MotorType.kBrushless);
        rightSpin = new CANSparkMax(KOrientationRightMotorID, MotorType.kBrushless);

        extension = new TalonSRX(KOrientationMotorExtensionID);
        
        DoorControl = new DigitalInput (KOrientationkDoorControlID);
        HallEffectSensorIn = new DigitalInput(KOrientationMagSensorInID);
        HallEffectSensorOut = new DigitalInput(KOrientationMagSensorOutID);

        leftSpin.setIdleMode(IdleMode.kBrake);
        rightSpin.setInverted(KOrientationRightMotorReversed);
        // extension.setIdleMode(IdleMode.kBrake);
        
        
    }

    /**
     * This command makes both motors move, because although it looks like only one moves, the follow() command is used on the right motor earlier.
     */
    public void moveOrientationLeftandRightMotors() {
        if (orientationMode) {
            leftSpin.set(0.5);
        }
        else if (!orientationMode) {
            leftSpin.set(0.5);
        }
    }

    /**
     * This command makes both motors move, because although it looks like only one moves, the follow() command is used on the right motor earlier.
     */
    public void moveOrientationLeftandRightMotors(double speed) {
        leftSpin.set(speed);
        rightSpin.set(speed);
    }

    public void moveOrientationMotorExtension(double speed) {
        extension.set(ControlMode.PercentOutput, speed);
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
        leftSpin.set(0);
    }

    public void stopOrientationMotorExtension() {
        extension.set(ControlMode.PercentOutput, 0);
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
