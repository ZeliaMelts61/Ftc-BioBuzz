package org.firstinspires.ftc.teamcode.robot;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;

import static java.lang.Math.atan2;
import androidx.annotation.NonNull;
import java.util.Set;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.mechanisms.*;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import org.firstinspires.ftc.teamcode.data.Alliance;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;
import dev.nextftc.robot.Telemetry;

/**
 * This is the Zelia Robot (the main robot class).
 * Almost all code goes through this class.
 * This class implements NextRobot, which means that the library will automatically construct it at startup.
 * Inside the main robot class there are mechanisms (E.g. drivetrain and intake),
 * each mechanism controls a different part of the robot.
 * Mechanisms are controlled via commands. Only one command can be using a mechanism at once.
 * If another command attempts to use the mechanism it will either cancel itself or cancel the
 * command currently using the mechanism.
 */
public class ZeliaRobot implements NextRobot {
    ElapsedTime telemetryTimer = new ElapsedTime();

    private Follower follower = null;

    private final Flywheel flywheel = new Flywheel();
    private final Intake intake = new Intake();
    private final Windmill windmill = new Windmill();
    private final Drivetrain drivetrain = new Drivetrain();

    private Alliance alliance = Alliance.NONE;

    /** NextRobot requires an empty constructor to be able to automatically create this class. */
    public ZeliaRobot(){}

    /**
     * Gets the flywheel mechanism
     * @return The flywheel mechanism
     */
    public Flywheel getFlywheel() {
        return flywheel;
    }

    /**
     * Gets the intake mechanism
     * @return The intake mechanism
     */
    public Intake getIntake() {
        return intake;
    }

    /**
     * Gets the windmill mechanism
     * @return The windmill mechanism
     */
    public Windmill getWindmill(){
        return windmill;
    }

    /**
     * Gets the drivetrain mechanism
     * @return The drivetrain mechanism
     */
    public Drivetrain getDrivetrain(){
        return drivetrain;
    }

    /**
     * Sets the Alliance color
     * @param alliance The Alliance that the robot is currently a part of.
     */
    public void setAlliance(Alliance alliance){
        this.alliance = alliance;
    }

    /**
     * Gets the Alliance color
     * @return The Alliance that the robot is currently a part of.
     */
    public Alliance getAlliance(){
        return alliance;
    }

    /**
     * Gets the follower. If there is currently no follower it creates one.
     * @return The follower
     */
    public Follower getFollower() {
        if (follower == null) {
            follower = Constants.create(dev.nextftc.hardware.RobotController.hardwareMap());
            follower.setPose(new Pose(0, 0, 0));
            drivetrain.setFollower(follower);
        }

        return follower;
    }

    /**
     * Initializes the robot fully.
     * IT IS IMPORTANT THAT THIS IS CALLED, without it a lot of stuff will break.
     */
    public void init() {
        drivetrain.setFollower(getFollower());
        schedule(
            infinite(this::loop)
        );
    }

    /**
     * The main loop. Currently, it only needs to update telemetry.
     */
    private void loop() {
        updateTelemetry();
    }

    /**
     * Updates the telemetry of the robot
     * Because of limited computation power this function will only update
     * the telemetry when it has been more than the TELEMETRY_UPDATE_MS.
     * It will update telemetry every loop if TelemetryConstants.debugMode is true.
     */
    private void updateTelemetry(){
        if (telemetryTimer.milliseconds() < RobotConstants.TelemetryConstants.TELEMETRY_UPDATE_MS && !RobotConstants.TelemetryConstants.debugMode) {
            return;
        }
        telemetryTimer.reset();

        Telemetry.log("Intake Roller Throttle", intake.getRollerThrottle());
        Telemetry.log("Intake Left Servo Throttle", intake.getLeftServoThrottle());
        Telemetry.log("Intake Right Servo Throttle", intake.getRightServoThrottle());
        Telemetry.log("");
        Telemetry.log("Flywheel Velocity (RPM)", flywheel.getCurrentRPM());
        Telemetry.log("Flywheel Target Velocity (RPM)", flywheel.getTargetRPM());
        Telemetry.log("Flywheel Velocity Error (RPM)", flywheel.getCurrentRPM() - flywheel.getTargetRPM());
        Telemetry.log("Flywheel Within Tolerance", flywheel.isReady());
        Telemetry.log("Flywheel Throttle", flywheel.getThrottle());
        Telemetry.log("Flywheel Activated", flywheel.getActivated());
        Telemetry.log("");
        Telemetry.log("Windmill Servo Throttle", windmill.getThrottle());
        Telemetry.log("");
        Telemetry.log("Drivetrain Velocity", follower.velocity());
        Telemetry.log("Drivetrain State", follower.mode().name());
        Telemetry.log("");
        Telemetry.log("Alliance", alliance.name());
        Telemetry.log("drive comm", drivetrain.getDefaultCommand().isScheduled());
        Telemetry.update();
    }


    /**
     * Returns a Command that runs the flywheel, the intake, and the windmill reverse at max speed.
     * This command will not end on its own.
     * @return A command that runs intake, flywheel, and windmill in reverse.
     */
    public Command runEverythingReverse(){
        return parallel(
                flywheel.deactivateCommand(),
                flywheel.setThrottleCommand(-1),
                intake.outtakeCommand(),
                windmill.reverseCommand()
        ).setEnd((e) -> {
                flywheel.setThrottleCommand(0);
                intake.stop();
                windmill.stop();
        });
    }

    /**
     * Sets the default drivetrain command to manual field oriented drive.
     * @param driveGamepad The gamepad that will be driving the robot.
     */
    public void startDrive(Gamepad driveGamepad){
        drivetrain.setDefaultCommand(
            drivetrain.manualRobotOrientedCommand(
                ()->driveGamepad.left_stick_y,
                ()->driveGamepad.left_stick_x,
                ()->driveGamepad.right_stick_x));
        drivetrain.getDefaultCommand().schedule();
    }

    /**
     * Returns a command that prepares the robot to shoot.
     * It stops the windmill, sets the intake to half power, and sets the
     * flywheel velocity based on distance to the hive.
     * THIS COMMAND DOES NOT STOP ITS MECHANISMS WHEN IT ENDS.
     * @return A command that prepares the robot to shoot
     */
    public Command setupShoot(){
        return parallel(
                windmill.stopCommand(),
                intake.setIntakeThrottleCommand(0.5),
                flywheel.setVelocityFromDistanceCommand(drivetrain.distanceToHive(alliance)),
                flywheel.activateCommand());

    }

    /**
     * Returns a command that will begin shooting.
     * If you can, you should call {@link #setupShoot()} a few seconds before this command.
     * It sets the flywheel velocity based on its distance from the hive.
     * Then it checks if the flywheel is within its speed tolerance.
     * If it is the windmill is powered on, if not the windmill is stopped.
     * @return a command that begins shooting.
     */
    public Command runShoot(){
        return infinite(()->{
            flywheel.setVelocityFromDistance(drivetrain.distanceToHive(alliance));

            if(flywheel.isReady()) {
                windmill.forward();
            } else windmill.stop();
        }).setEnd((e)-> {
            flywheel.deactivate();
            intake.stop();
            windmill.stop();
        }).requiring(flywheel,windmill,intake).setStart(()->{
            windmill.stop();
            intake.setAllThrottle(0.5);
            flywheel.setVelocityFromDistance(drivetrain.distanceToHive(alliance));
            flywheel.activate();
        });
    }


    /**
     * Wrapper for {@link #runShoot()}
     * @return runShoot()
     */
    public Command shoot(){
        return runShoot();
    }

    /**
     * Returns a command that will (in order),
     * 1. Setup shoot, stop windmill, set the intake to half power, begin speeding up the flywheel
     * 2. Path the robot to the shoot pose whist speeding up the flywheel.
     * 3. Run shoot, sets flywheel velocity based on distance, checks if the flywheel is up to speed,
     * if it is spin the windmill and launch a ball, if not stop the windmill
     * @return A command that sets up a shoot, drives robot to shoot pose, and shoots.
     */
    public Command driveToShootSpotThenShoot(){
        return sequential(
                setupShoot(),
                race(
                        drivetrain.pathToShootPoseCommand(alliance),
                        flywheel.setVelocityFromDistanceContinuousCommand(() -> drivetrain.distanceToHive(alliance))),
                runShoot());
    }

    // Not entirely sure if this math works out

    /**
     * Returns a command that will (in order),
     * 1. Setup shoot, stop windmill, set the intake to half power, begin speeding up the flywheel
     * 2. Rotate the robot to face the hive while still allowing driver control
     * 3. Begin shooting.
     * <p>
     * FIXME: 9/27/2026
     * There are a few flaws with this currently.
     * I do not check if the robot is actually at the correct angle
     * to shoot into the hive before it starts shooting
     * and I'm not entirely sure if this math checks out.
     * </p>
     * @param driveGamepad The gamepad that the drives the robot so they can still drive
     *                    while the robot rotates itself
     * @return A command that sets up shooting, rotates to the hive, and begins shooting.
     */
    public Command turnToTargetAndShoot(Gamepad driveGamepad){
        return sequential(setupShoot(), parallel(
                drivetrain.manualHoldAngleFieldOrientedCommand(
                        ()->driveGamepad.left_stick_y,
                        ()->driveGamepad.left_stick_x,
                        ()->driveGamepad.right_stick_x,
                        ()->{
                            Pose hivePose = drivetrain.closestHivePose(follower.pose(), alliance);
                            Pose robotPose = follower.pose();
                            double dx = hivePose.x() - robotPose.x();
                            double dy = hivePose.y() - robotPose.y();
                            return atan2(dy, dx);
                        }
                ),
                shoot()));

    }

    public Command continuousFlywheelSpeedControl(){
        return flywheel.setVelocityFromDistanceContinuousCommand(()->drivetrain.distanceToHive(alliance));
    }

    public Command shootAtRPM(double rpm){
        return infinite(()->{
            if(flywheel.isReady()) {
                windmill.forward();
            } else windmill.stop();
        }).setEnd((e)-> {
            flywheel.deactivate();
            intake.stop();
            windmill.stop();
        }).requiring(flywheel,windmill,intake).setStart(()->{
            windmill.stop();
            intake.setAllThrottle(0.5);
            flywheel.setTargetRPM(rpm);
            flywheel.activate();
        });
    }






    @NonNull
    @Override
    public Set<Mechanism> getMechanisms() {
        return Set.of(intake, flywheel, windmill, drivetrain);
    }

}
