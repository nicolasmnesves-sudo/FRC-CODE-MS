package frc.robot.Joysticks;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class KeyboardController extends CommandGenericHID {

    public KeyboardController(int port) {
        super(port);
    }

    public Trigger getZTrigger() { return button(1); }
    public Trigger getXTrigger() { return button(2); }
    public Trigger getCTrigger() { return button(3); }
    public Trigger getVTrigger() { return button(4); }
    public Trigger getBTrigger() { return button(5); }
    public Trigger getNTrigger() { return button(6); }
    public Trigger getMTrigger() { return button(7); }
    public Trigger getFTrigger() { return button(8); }
    public Trigger getGTrigger() { return button(9); }
    public Trigger getHTrigger() { return button(10); }
}