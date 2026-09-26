package org.firstinspires.ftc.teamcode.robot;


import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;

import static java.lang.Math.atan2;

import androidx.annotation.NonNull;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotConstants.MatchConstants.*;
import org.firstinspires.ftc.teamcode.mechanisms.*;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.Set;

import dev.nextftc.control.geometry.Twist2d;
import dev.nextftc.hardware.RobotController;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;
import dev.nextftc.robot.Telemetry;

public class ZeliaRobot implements NextRobot {
    private Follower follower = null;

    ElapsedTime telemetryTimer = new ElapsedTime();



    private final Flywheel flywheel = new Flywheel();
    private final Intake intake = new Intake();
    private final Windmill windmill = new Windmill();
    private Drivetrain drivetrain = new Drivetrain();
    private ALLIANCE_COLOR alliance;

    public ZeliaRobot(){}


    public Flywheel getFlywheel() {
        return flywheel;
    }

    public Intake getIntake() {
        return intake;
    }

    public Windmill getWindmill(){
        return windmill;
    }

    public Drivetrain getDrivetrain(){return drivetrain;}

    public void setAlliance(ALLIANCE_COLOR alliance){
        this.alliance = alliance;
    }

    public ALLIANCE_COLOR getAlliance(){
        return alliance;
    }

    public Follower getFollower() {
        if (follower == null) {
            follower = Constants.create(dev.nextftc.hardware.RobotController.hardwareMap());
            drivetrain=new Drivetrain(follower);
        }

        return follower;
    }

    public void init(Follower follower) {
        this.follower=follower;
        drivetrain = new Drivetrain(follower);
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

        Telemetry.log("Intake Roller Power", intake.getRollerThrottle());
        Telemetry.log("Intake Left Servo Power", intake.getLeftServoThrottle());
        Telemetry.log("Intake Right Servo Power", intake.getRightServoThrottle());
        Telemetry.log("");
        Telemetry.log("Flywheel Velocity (RPM)", flywheel.getCurrentRPM());
        Telemetry.log("Flywheel Target Velocity (RPM)", flywheel.getTargetRPM());
        Telemetry.log("Flywheel Velocity Error (RPM)", flywheel.getCurrentRPM() - flywheel.getTargetRPM());
        Telemetry.log("Flywheel Within Tolerance", flywheel.isReady());
        Telemetry.log("Flywheel Power", flywheel.getThrottle());
        Telemetry.log("Flywheel Activated", flywheel.getActivated());
        Telemetry.log("");
        Telemetry.update();
    }


    public Command runEverythingBackwards(){
        return parallel(
                flywheel.setThrottleCommand(-1),
                intake.outtakeCommand(),
                windmill.reverseCommand()
        ).setEnd((e) -> {
                flywheel.setThrottleCommand(0);
                intake.stop();
                windmill.stop();
            }
        );
    }

    public void startDrive(Gamepad driveGamepad){
        drivetrain.setDefaultCommand(
            drivetrain.manualFieldOrientedCommand(
                ()->driveGamepad.left_stick_y,
                ()->driveGamepad.left_stick_x,
                ()->driveGamepad.right_stick_x)).schedule();
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

    // Not entirely sure if this math works out
    public Command turnToTargetAndShoot(Gamepad driveGamepad){
        return parallel(
                drivetrain.manualHoldAngleFieldOrientedCommand(
                        ()->driveGamepad.left_stick_y,
                        ()->driveGamepad.left_stick_x,
                        ()->driveGamepad.right_stick_x,
                        ()->{
                            Pose hivePose = drivetrain.closestHivePose(follower.pose());
                            Pose robotPose = follower.pose();
                            double dy=hivePose.x() - robotPose.x();
                            double dx=hivePose.y() - robotPose.y();
                            return atan2(dy, dx);
                        }
                ),
                shoot()
        );
    }





    @NonNull
    @Override
    public Set<Mechanism> getMechanisms() {
        return Set.of(intake, flywheel, windmill);
    }

}
