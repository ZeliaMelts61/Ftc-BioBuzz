package org.firstinspires.ftc.teamcode.opmodes.utils;


import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Scheduler;

import org.firstinspires.ftc.teamcode.robot.ZeliaRobot;

import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextUtility;
import dev.nextftc.robot.triggers.CommandGamepad;
import dev.nextftc.robot.triggers.Trigger;

// --------------- Controller Mapping ----------------
// Game Pad 1:
//  Left Bumper = Intake
//  left trigger = shoot
//  right trigger = shoot at rpm
//  y = shoot at rpm
//  dpad down = outtake
//  dpad up = intake
//  dpad left = windmill forward
//  dpad right = windmill reverse
//  start = reset heading
//  back = stop everything
//  a = PATH?
// Game Pad 2:
//  dpad right = intake right servo
//  dpad left = intake left servo
@NextUtility(name = "Pose Test")
public class poseTest extends NextOpMode {
    private final ZeliaRobot robot;

    public poseTest(ZeliaRobot robot) {
        super(robot);
        this.robot = robot;
        Scheduler.reset();
    }

    @Override
    public void start() {
        Trigger.Companion.getDefaultEventLoop().clear();



        robot.init(); // MUST CALL BEFORE RUNNING STUFF
    }

    @Override
    public void periodic() {
        robot.getFollower().update();
    }

    @Override
    public void end() {
        robot.getIntake().stop();
        robot.getFlywheel().deactivate();
        robot.getWindmill().stop();
        robot.getDrivetrain().stop();

    }
}
