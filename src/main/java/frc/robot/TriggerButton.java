package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TriggerButton extends Trigger{
    public TriggerButton(XboxController controller){
        super(() -> controller.getLeftTriggerAxis() >= 0.75);
    }
}
