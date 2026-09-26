package org.firstinspires.ftc.teamcode.opmodes.teleop;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.robot.ZeliaRobot;
import dev.nextftc.robot.NextRobot;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;

@NextTeleop(name = "testOpMode")
public class testOpMode extends NextOpMode {
    ZeliaRobot robot;
    public testOpMode(@NonNull NextRobot robot) {
        super(robot);
        this.robot=(ZeliaRobot)robot;
    }

    public void init() {

    }

    public void loop() {
        telemetry.addLine("running");
    }
}
