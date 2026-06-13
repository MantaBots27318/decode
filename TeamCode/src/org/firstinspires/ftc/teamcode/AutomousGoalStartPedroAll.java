package org.firstinspires.ftc.teamcode;
/*tis is a new auto*/

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous(name = "Goal Start - Blue", group = "Autonomous")
public class AutomousGoalStartPedroAll extends OpMode {
        Follower follower;
        public PathChain MainChain;

        public PathChain Paths(Follower follower) {
            MainChain = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(19.745, 120.461),
                                    new Pose(52.480, 88.623)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(135))
                    .addPath(
                            new BezierCurve(
                                    new Pose(52.480, 88.623),
                                    new Pose(66.539, 80.119),
                                    new Pose(36.950, 82.591)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .addPath(
                            new BezierLine(
                                    new Pose(36.950, 82.591),
                                    new Pose(15, 82.410)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .addPath(
                            new BezierLine(
                                    new Pose(15, 82.410),
                                    new Pose(51.990, 88.808)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .addPath(
                            new BezierCurve(
                                    new Pose(51.990, 88.808),
                                    new Pose(73.933, 54.475),
                                    new Pose(38.149, 58.845)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .addPath(
                            new BezierLine(
                                    new Pose(38.149, 58.845),
                                    new Pose(12.812, 59.218)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .addPath(
                            new BezierLine(
                                    new Pose(12.812, 59.218),
                                    new Pose(52.114, 87.869)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .addPath(
                            new BezierCurve(
                                    new Pose(52.114, 87.869),
                                    new Pose(77.715, 33.223),
                                    new Pose(38.007, 34.351)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .addPath(
                            new BezierLine(
                                    new Pose(38.007, 34.351),
                                    new Pose(11.237, 35.135)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .addPath(
                            new BezierLine(
                                    new Pose(11.237, 35.135),
                                    new Pose(51.703, 88.985)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .addPath(
                            new BezierLine(
                                    new Pose(51.703, 88.985),
                                    new Pose(59.919, 100.837)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135))
                    .build();
            return MainChain;

        }
        @Override
        public void init(){
                follower = Constants.createFollower(hardwareMap);
                follower.setPose(new Pose(19.745, 120.461, Math.toRadians(143)));
        }
        public void start() {
                follower.followPath(Paths(follower));   // kicks it off
        }
        @Override
        public void loop() {
                follower.update();             // MUST be called every iteration
                // optional telemetry

        }
}
