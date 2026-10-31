package org.firstinspires.ftc.teamcode.opmodes.auto.commands;


import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.opmodes.auto.paths.PathsAndPoses;
import org.firstinspires.ftc.teamcode.robot.ZeliaRobot;

public class AutoCommands {
//    private final Follower follower;
    private final PathsAndPoses paths;
    private final ZeliaRobot robot;

    public AutoCommands(ZeliaRobot robot, PathsAndPoses paths){
        this.robot = robot;
        this.paths = paths;
    }
    public CommandBuilder runPath(Path path){
        return follow(robot.getFollower(), path);
    }

    /**
     * Returns a shoot Command that last the specified amount of time
     * @param ms The amount of milliseconds the shooting should last
     * @return a shoot Command that last the specified amount of time
     */
    public Command timedShoot(double ms) {
        return robot.shoot().raceWith(waitMs(ms));
    }

    public Command followPath(Path path){
        return robot.getDrivetrain().followPathCommand(path);
    }

    public Command intakePath(Path path){
        return sequential(
            robot.getIntake().intakeCommand(),
            followPath(path),
            robot.getIntake().stopIntakeCommand()
        );
    }

    public Command timedIntake(double ms){
        return sequential(
                robot.getIntake().intakeCommand(),
                waitMs(ms),
                robot.getIntake().stopIntakeCommand()
        );
    }


}

