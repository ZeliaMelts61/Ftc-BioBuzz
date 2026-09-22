package org.firstinspires.ftc.teamcode;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;

import androidx.annotation.AnimRes;
import androidx.annotation.AnimatorRes;
import androidx.annotation.AnyRes;
import androidx.annotation.AnyThread;
import androidx.annotation.ArrayRes;
import androidx.annotation.BinderThread;
import androidx.annotation.CallSuper;
import androidx.annotation.ColorRes;
import androidx.annotation.Dimension;
import androidx.annotation.GuardedBy;
import androidx.annotation.InspectableProperty;
import androidx.annotation.NonNull;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Windmill;

import java.util.Set;

import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;
import kotlin.Suppress;

public class Robot implements NextRobot {

    private HardwareMap hardwareMap;
    private Gamepad gamepad1;
    private Gamepad gamepad2;
    private Telemetry telemetry;
    private Follower follower;
    private VoltageSensor voltageSensor;

    private Drivetrain drivetrain;
    private Intake intake;
    private Flywheel flywheel;
    private Windmill windmill;


    ElapsedTime telemetryTimer = new ElapsedTime();


//    public Robot(HardwareMap hardwareMap,Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, Follower follower){
//        this.hardwareMap=hardwareMap;
//        this.gamepad1=gamepad1;
//        this.gamepad2=gamepad2;
//        this.telemetry=telemetry;
//        this.follower=follower;
//
//        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
//
//        drivetrain = new Drivetrain(follower);
//        intake = new Intake(hardwareMap);
//        flywheel = new Flywheel(hardwareMap, voltageSensor);
//        windmill = new Windmill(hardwareMap);
//    }

    public void init() {
        schedule(
            infinite(this::loop)
        );
    }

    private void loop() {
        updateTelemetry();
    }

    private void updateTelemetry(){
        if (telemetryTimer.milliseconds() < RobotConstants.TelemetryConstants.TELEMETRY_UPDATE_MS && !RobotConstants.TelemetryConstants.debugMode) {
            return;
        }
        telemetryTimer.reset();

        telemetry.addData("Intake Roller Power", intake.getRollerThrottle());
        telemetry.addData("Intake Left Servo Power", intake.getLeftServoThrottle());
        telemetry.addData("Intake Right Servo Power", intake.getRightServoThrottle());
        telemetry.addLine();
        telemetry.addData("Flywheel Velocity (RPM)", flywheel.getCurrentRPM());
        telemetry.addData("Flywheel Target Velocity (RPM)", flywheel.getTargetRPM());
        telemetry.addData("Flywheel Velocity Error (RPM)", flywheel.getCurrentRPM() - flywheel.getTargetRPM());
        telemetry.addData("Flywheel Within Tolerance", flywheel.isReady());
        telemetry.addData("Flywheel Power", flywheel.getThrottle());
        telemetry.addData("Flywheel Activated", flywheel.getActivated());
        telemetry.addLine();



    }


    public HardwareMap getHardwareMap(){
        return hardwareMap;
    }

    public Telemetry getTelemetry(){
        return telemetry;
    }

    public Gamepad getGamepad1(){
        return gamepad1;
    }

    public Gamepad getGamepad2(){
        return gamepad2;
    }

    public Follower getFollower(){
        return follower;
    }


    public Command setupShoot(){
        return parallel(
                windmill.stopCommand(),
                intake.setIntakeThrottleCommand(0.5),
                flywheel.setVelocityFromDistanceCommand(drivetrain.distanceToHive()),
                flywheel.activateCommand())
            .setEnd((e)-> {
                flywheel.deactivate();
                intake.stop();
                windmill.stop();
            });
    }

    public Command runShoot(){
        return infinite(()->{
            flywheel.setVelocityFromDistance(drivetrain.distanceToHive());

            if(flywheel.isReady()) {
                windmill.forward();
            } else windmill.stop();
        }).setEnd((e)-> {
            flywheel.deactivate();
            intake.stop();
            windmill.stop();
        });
    }


    public Command shoot(){
        return sequential(
                setupShoot(),
                runShoot());
    }

    public Command driveToShootSpotThenShoot(){
        return sequential(
                setupShoot(),
                drivetrain.pathToShootPoseCommand(),
                runShoot());
    }

    @NonNull
    @Override
    public Set<Mechanism> getMechanisms() {
        return Set.of(drivetrain,flywheel,intake,windmill);
    }
}

