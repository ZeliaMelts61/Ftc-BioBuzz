import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.onInterrupt;
import static com.pedropathing.ivy.groups.Groups.parallel;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Windmill;
import org.firstinspires.ftc.teamcode.util.MathHelpers;

public class Robot {

    private HardwareMap hardwareMap;
    private Gamepad gamepad1;
    private Gamepad gamepad2;
    private Telemetry telemetry;
    private Follower follower;
    private VoltageSensor voltageSensor;

    private Intake intake;
    private Flywheel flywheel;
    private Windmill windmill;


    ElapsedTime telemetryTimer = new ElapsedTime();


    public Robot(HardwareMap hardwareMap,Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, Follower follower){
        this.hardwareMap=hardwareMap;
        this.gamepad1=gamepad1;
        this.gamepad2=gamepad2;
        this.telemetry=telemetry;
        this.follower=follower;

        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();

        intake = new Intake(hardwareMap);
        flywheel = new Flywheel(hardwareMap, voltageSensor);
        windmill = new Windmill(hardwareMap);
    }

    public void init() {
        schedule(
            infinite(this::loop)
        );
    }

    private void loop() {
        flywheel.update();
        updateTelemetry();
    }

    private void updateTelemetry(){
        if (telemetryTimer.milliseconds() < RobotConstants.TelemetryConstants.TELEMETRY_UPDATE_MS && !RobotConstants.TelemetryConstants.debugMode) {
            return;
        }
        telemetryTimer.reset();

        telemetry.addData("Intake Roller Power", intake.getRollerPower());
        telemetry.addData("Intake Left Servo Power", intake.getLeftServoPower());
        telemetry.addData("Intake Right Servo Power", intake.getRightServoPower());
        telemetry.addLine();
        telemetry.addData("Flywheel Velocity (RPM)", flywheel.getCurrentRPM());
        telemetry.addData("Flywheel Target Velocity (RPM)", flywheel.getTargetRPM());
        telemetry.addData("Flywheel Velocity Error (RPM)", flywheel.getCurrentRPM() - flywheel.getTargetRPM());
        telemetry.addData("Flywheel Velocity Error (TPS)", flywheel.getCurrentAngularVel() - flywheel.getTargetAngularVelocity());
        telemetry.addData("Flywheel Within Tolerance", flywheel.isReady());
        telemetry.addData("Flywheel Power", flywheel.getPower());
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

    public Command setIntakePower(double power) {
        return instant(() -> intake.setAllPower(power));
    }

    public Command intake(){
        return setIntakePower(1);
    }

    public Command outtake(){
        return setIntakePower(-1);
    }

    public Command stopIntake(){
        return instant(()-> intake.stop());
    }

    public Command setFlywheelTargetRPM(double RPM){
        return instant(()-> flywheel.setTargetRPM(RPM));
    }

    public Command setFlywheelTargetTPS(double TPS){
        return instant(()-> flywheel.setTargetAngularVelocity(TPS));
    }

    public Command activateFlywheel(){
        return  instant(() -> flywheel.activate());
    }

    public Command deactivateFlywheel(){
        return  instant(() -> flywheel.deactivate());
    }

    // not entirely sure if this works
    public Command flywheelReverse(){
        return instant(() -> {
            flywheel.deactivate();
            flywheel.setPower(-1);
        });
    }

    public Command windmillForward(){
        return instant(() -> windmill.forward());
    }

    public Command windmillReverse(){
        return instant(() -> windmill.reverse());
    }

    public Command setWindmillPower(double power) {
        return instant(()-> windmill.setPower(power));
    }

    public Command stopWindmill(){
        return instant(() -> windmill.stop());
    }

    public Command sortaAutoShoot(){
        return parallel(
            Command.build()
                .requiring(intake, windmill, flywheel)
                .setStart(() -> {
                    flywheel.setTargetRPM(3000);
                    flywheel.activate();
                    intake.setAllPower(0.5);
                    windmill.stop();
                })
                .setExecute(() -> {
                    conditional(flywheel::isReady, windmillForward(), stopWindmill());
                })
                .setDone(()->false),
            onInterrupt(() -> {
                flywheel.deactivate();
                intake.stop();
                windmill.stop();
            }));
    }
}

