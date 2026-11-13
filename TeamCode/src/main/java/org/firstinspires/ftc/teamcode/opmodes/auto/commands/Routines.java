package org.firstinspires.ftc.teamcode.opmodes.auto.commands;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.opmodes.auto.paths.PathsAndPoses;
import org.firstinspires.ftc.teamcode.robot.ZeliaRobot;

public class Routines {
    private final ZeliaRobot robot;
    private final PathsAndPoses paths;
    private final AutoCommands commands;

    public Routines(ZeliaRobot robot){
        this.robot = robot;
        this.paths = new PathsAndPoses(robot.getAlliance());
        this.commands = new AutoCommands(robot, paths);
    }

    public Command simpleShootDrive(){
        robot.getFollower().setPose(PathsAndPoses.startFacingUpHive);
        return sequential(
                commands.timedShoot(15000),
                robot.getDrivetrain().manualArcadeCommand(()->-0.5,()->0)
                        .raceWith(waitMs(2000))
        );
    }

    public Command waitSimpleShootDrive(){
        robot.getFollower().setPose(PathsAndPoses.startFacingDownHive);
        return sequential(
                waitMs(10000),
                commands.timedShoot(15000),
                robot.getDrivetrain().manualArcadeCommand(()->-0.5,()->0)
                        .raceWith(waitMs(2000))
        );
    }

    public Command shootPark(){
        robot.getFollower().setPose(PathsAndPoses.startFacingUpHive);
        return sequential(
                commands.timedShoot(10000),
                commands.followPath(paths.startShootToPark())
        );
    }

    public Command shootPickup() {
        robot.getFollower().setPose(PathsAndPoses.startFacingUpHive);
        return sequential(
                commands.timedShoot(10000),
                commands.followPath(paths.startToGardenLineup()),
                commands.intakePath(paths.gardenPickup()),
                commands.timedIntake(500)
        );
    }

    public Command shootPickupPark(){
        robot.getFollower().setPose(PathsAndPoses.startFacingUpHive);
        return sequential(
                commands.timedShoot(10000),
                commands.followPath(paths.startToGardenLineup()),
                commands.intakePath(paths.gardenPickup()),
                commands.timedIntake(500),
                commands.followPath(paths.gardenToPark())
        );
    }

    public Command full2CycleAuto() {
        robot.getFollower().setPose(PathsAndPoses.startFacingUpHive);
        return sequential(
                commands.timedShoot(5000),
                commands.followPathTimeout(paths.startToGardenLineup(),2),
                commands.intakePathTimeout(paths.gardenPickup(),3),
                commands.timedIntake(500),
                commands.followPathTimeout(paths.gardenToShoot2(),3)
                        .with(robot.setupShoot()),
                commands.timedShoot(5000),
                commands.intakePathTimeout(paths.shoot2ToPark(),4)
        );
    }

    public Command waitFarShootPark(){
        robot.getFollower().setPose(PathsAndPoses.startFacingDownHive);
        return sequential(
                waitMs(8000),
                robot.setupShoot(),
                waitMs(2000),
                commands.timedShoot(10000),
                commands.followPath(paths.waitShootPark())
        );
    }

    public Command waitFarShootPickup(){
        robot.getFollower().setPose(PathsAndPoses.startFacingDownHive);
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
        robot.getFollower().setPose(PathsAndPoses.startFacingDownHive);
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
