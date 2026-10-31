package org.firstinspires.ftc.teamcode.opmodes.auto.runnables.red.close;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import org.firstinspires.ftc.teamcode.data.Alliance;
import org.firstinspires.ftc.teamcode.opmodes.auto.commands.Routines;
import org.firstinspires.ftc.teamcode.robot.ZeliaRobot;

import dev.nextftc.robot.opmode.NextAutonomous;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.triggers.Trigger;

@NextAutonomous(name = "RedCloseShootPickupPark")
public class RedCloseShootPickupPark extends NextOpMode {

    private final ZeliaRobot robot;
    private Routines routines;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private Command autoRoutine;

    public RedCloseShootPickupPark(ZeliaRobot robot){
        super(robot);
        this.robot = robot;
        robot.setAlliance(Alliance.RED);
        Scheduler.reset();
    }

    @Override
    public void start(){
        Trigger.Companion.getDefaultEventLoop().clear();
        robot.init();

        routines = new Routines(robot);

        autoRoutine = routines.shootPickupPark();
        autoRoutine.schedule(); // Make sure you schedule it
    }


    public void end() {
        robot.getIntake().stop();
        robot.getFlywheel().deactivate();
        robot.getWindmill().stop();
        robot.getDrivetrain().stop();

        autoRoutine.cancel();
    }
}
