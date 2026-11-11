package org.firstinspires.ftc.teamcode.mechanisms.touchpad;

import com.qualcomm.robotcore.hardware.Gamepad;
import dev.nextftc.robot.triggers.RangeTrigger;
import dev.nextftc.robot.triggers.Trigger;

public class Touchpad {
    public final RangeTrigger x;
    public final RangeTrigger y;
    public final Trigger click;

    // Accept the raw hardware Gamepad stream directly
    public Touchpad(Gamepad gamepad) {
        this.x = new RangeTrigger(() -> {
            return gamepad.touchpad_finger_1 ? (double) gamepad.touchpad_finger_1_x : 0.0;
        });

        this.y = new RangeTrigger(() -> {
            return gamepad.touchpad_finger_1 ? (double) gamepad.touchpad_finger_1_y : 0.0;
        });

        this.click = new Trigger(() -> gamepad.touchpad_finger_1);
    }
}
