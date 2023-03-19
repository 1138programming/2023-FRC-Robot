package frc.robot.subsystems;

import static frc.robot.Constants.*;

import javax.sound.sampled.Line;

import com.revrobotics.CANSparkMax; // Covers Neo's
import com.revrobotics.CANSparkMaxLowLevel.MotorType; // Cover's Neo's
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.DigitalInput; // Sensors
import edu.wpi.first.wpilibj.Ultrasonic;

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
    
    private DigitalInput doorControl;
    private DigitalInput HallEffectSensorIn;
    private DigitalInput HallEffectSensorOut;

    private Ultrasonic doorChecker;
    MedianFilter doorCheckerFilter;

    private boolean orientationMode;

    public Orientation() {

        leftSpin = new CANSparkMax(KOrientationLeftMotorID, MotorType.kBrushless);
        rightSpin = new CANSparkMax(KOrientationRightMotorID, MotorType.kBrushless);

        leftSpin.setIdleMode(IdleMode.kCoast);
        rightSpin.setIdleMode(IdleMode.kCoast);

        extension = new TalonSRX(KOrientationMotorExtensionID);
        
        doorControl = new DigitalInput (KOrientationkDoorControlID);
        HallEffectSensorIn = new DigitalInput(KOrientationMagSensorInID);
        HallEffectSensorOut = new DigitalInput(KOrientationMagSensorOutID);

        doorChecker = new Ultrasonic(KOrientationRangeOut, KOrientationRangeIn);
        doorCheckerFilter = new MedianFilter(6);
        Ultrasonic.setAutomaticMode(true);

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

    // public boolean getDoorSensor() {
    //     return doorControl.get();
    // }

    public boolean  
    getMagSensorOut() {
        return !HallEffectSensorOut.get();
    }

    public boolean getMagSensorIn(){
        return !HallEffectSensorIn.get();
    }
    public double getDoorRange() {
        return doorCheckerFilter.calculate(doorChecker.getRangeInches());
    }
    public boolean getDoorSensor() {
        return getDoorRange() < 12;
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("range", getDoorRange());
        SmartDashboard.putBoolean("range being detected", getDoorRange() > 19 && getDoorRange() < 22);
        SmartDashboard.putBoolean("CLOSE", getDoorSensor());
        SmartDashboard.putBoolean("Out", getMagSensorOut());
        SmartDashboard.putBoolean("IN", getMagSensorIn());
    }
}
