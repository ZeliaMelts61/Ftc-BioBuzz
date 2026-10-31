package org.firstinspires.ftc.teamcode.opmodes.auto.commands;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;

import org.firstinspires.ftc.teamcode.opmodes.auto.paths.PathsAndPoses;
import org.firstinspires.ftc.teamcode.robot.ZeliaRobot;

public class Routines {
    private ZeliaRobot robot;
    private final PathsAndPoses paths;
    private final AutoCommands commands;

    public Routines(ZeliaRobot robot){
        this.robot = robot;
        this.paths = new PathsAndPoses(robot.getAlliance());
        this.commands = new AutoCommands(robot, paths);
    }

    public Command shootPark(){
        robot.getFollower().setPose(paths.startFacingUpHive);
        return sequential(
                commands.timedShoot(10000),
                commands.followPath(paths.startShootToPark())
        );
    }

    public Command shootPickup() {
        robot.getFollower().setPose(paths.startFacingUpHive);
        return sequential(
                commands.timedShoot(10000),
                commands.followPath(paths.startToGardenLineup()),
                commands.intakePath(paths.gardenPickup()),
                commands.timedIntake(500)
        );
    }

    public Command shootPickupPark(){
        robot.getFollower().setPose(paths.startFacingUpHive);
        return sequential(
                commands.timedShoot(10000),
                commands.followPath(paths.startToGardenLineup()),
                commands.intakePath(paths.gardenPickup()),
                commands.timedIntake(500),
                commands.followPath(paths.gardenToPark())
        );
    }

    public Command full2CycleAuto() {
        robot.getFollower().setPose(paths.startFacingUpHive);
        return sequential(
                commands.timedShoot(5000),
                commands.followPath(paths.startToGardenLineup()),
                commands.intakePath(paths.gardenPickup()),
                commands.timedIntake(500),
                commands.followPath(paths.gardenToShoot2())
                        .with(robot.setupShoot()),
                commands.timedShoot(5000),
                commands.intakePath(paths.shoot2ToPark())
        );
    }

    public Command waitFarShootPark(){
        robot.getFollower().setPose(paths.startFacingDownHive);
        return sequential(
                waitMs(8000),
                robot.setupShoot(),
                waitMs(2000),
                commands.timedShoot(10000),
                commands.followPath(paths.waitShootPark())
        );
    }

    public Command waitFarShootPickup(){
        robot.getFollower().setPose(paths.startFacingDownHive);
        return sequential(
                waitMs(8000),
                robot.setupShoot(),
                waitMs(2000),
                commands.timedShoot(8000),
                commands.followPath(paths.farShootToPickup()),
                commands.intakePath(paths.gardenPickup()),
                commands.timedIntake(500)
        );
    }

    public Command waitFarShootPickupPark(){
        robot.getFollower().setPose(paths.startFacingDownHive);
        return sequential(
                waitMs(8000),
                robot.setupShoot(),
                waitMs(2000),
                commands.timedShoot(8000),
                commands.followPath(paths.farShootToPickup()),
                commands.intakePath(paths.gardenPickup()),
                commands.timedIntake(500),
                commands.followPath(paths.gardenToPark())
        );
    }





}
