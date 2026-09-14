package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.Paths;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.pedro.AutoCommands;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

import com.pedropathing.ivy.Scheduler;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Autonomous
public class MyFirstTestAuto extends OpMode {

    private Follower follower; // The Auto follower
    private final PoseFactory p = PoseFactory.degrees();


    @Override
    public void init() {
        Scheduler.reset(); // resets the Ivy command scheduler
        follower = Constants.create(hardwareMap); // Assigns the auto follower
        follower.setPose(AutoCommands.AutoPoses.testStartPose); // tells the follower where the robot starts on the field
        follower.update();

    }

    @Override
    public void start() {
        schedule(AutoCommands.AutoRoutines.testAutoRoutine(follower));
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        // telemetry
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
    }
}
