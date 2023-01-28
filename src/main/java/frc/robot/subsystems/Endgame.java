package frc.robot.subsystems;

import static frc.robot.Constants.*;

//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Endgame extends SubsystemBase {

    // Can be used if needed...
    // private DigitalInput endgameTopLimitSwitch;
    // private DigitalInput endgameBottomLimitSwitch;

    private Servo endgameLinearServoTop;
    private Servo endgameLinearServoBottom;
    

    public Endgame() {
        // endgameTopLimitSwitch = new DigitalInput(KLimitSwitchTop);
        // endgameBottomLimitSwitch = new DigitalInput(KLimitSwitchBottom);
        endgameLinearServoTop = new Servo(KLinearServoTop);
        endgameLinearServoBottom = new Servo(KLinearServoBottom);

        // Bounds copied from 2022 FRC robot.. the values are most likely wrong. Check with Patrick for values possibly
        endgameLinearServoTop.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
        endgameLinearServoTop.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
    }

    public void moveServo(double pos) {
        endgameLinearServoTop.set(pos);
        endgameLinearServoBottom.set(pos);
    }
}
