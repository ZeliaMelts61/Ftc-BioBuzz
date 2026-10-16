package org.firstinspires.ftc.teamcode.opmodes.auto.commands;


import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
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


}

