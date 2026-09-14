package org.firstinspires.ftc.teamcode.pedro;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class AutoCommands {
    public static class AutoPoses{
        public static final PoseFactory p = PoseFactory.degrees();
        public static final Pose testStartPose = p.of(24, 24, 0);
        public static final Pose testScorePose = p.of(48, 48, 90);
        public static final Pose testParkPose = p.of(72, 48, 90);
        public static final Pose testControlPose = p.of(36, 60, 45);
    }
    public static class AutoPaths{
        public static Path testStartToScore() {
            return line(AutoPoses.testStartPose, AutoPoses.testScorePose).linear(AutoPoses.testStartPose, AutoPoses.testScorePose);
        }
        public static Path testPark(){
            return line(AutoPoses.testScorePose, AutoPoses.testParkPose).linear(AutoPoses.testScorePose, AutoPoses.testParkPose);
        }
    }
    public static class ScoringCommands {
        public static Command shootPollen() {
            return new Command(); // ill figure that out later
        }
    }

    public static class AutoRoutines{
        public static Command testAutoRoutine(Follower follower) {
            return sequential(
                    follow(follower, AutoPaths.testStartToScore()),
                    // Add mechanism commands here.
                    ScoringCommands.shootPollen(),
                    follow(follower, AutoPaths.testPark())
            );
        }
    }
}
