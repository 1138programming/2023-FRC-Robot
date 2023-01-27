package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Endgame extends SubsystemBase {
DigitalInput endgameTopLimitSwitch;
DigitalInput endgameBottomLimitSwitch;

    public Endgame() {
        endgameTopLimitSwitch = new DigitalInput();
        endgameBottomLimitSwitch = new DigitalInput();
    }
}
