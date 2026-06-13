package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous(name = "Goal Start - Red", group = "Autonomous")
public class AutomousGoalStartPedroAllRed extends OpMode {
        Follower follower;
        public PathChain MainChain;

        public PathChain Paths(Follower follower) {
            MainChain = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(124.255, 120.461),
                                    new Pose(91.520, 87.623)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(45))
                    .addPath(
                            new BezierCurve(
                                    new Pose(91.520, 87.623),
                                    new Pose(77.461, 80.119),
                                    new Pose(105.050, 81.591)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .addPath(
                            new BezierLine(
                                    new Pose(105.050, 81.591),
                                    new Pose(129, 81.410)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .addPath(
                            new BezierLine(
                                    new Pose(129, 81.410),
                                    new Pose(92.010, 87.808)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .addPath(
                            new BezierCurve(
                                    new Pose(92.010, 87.808),
                                    new Pose(70.067, 54.475),
                                    new Pose(105.851, 57.845)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .addPath(
                            new BezierLine(
                                    new Pose(105.851, 57.845),
                                    new Pose(131.188, 58.218)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .addPath(
                            new BezierLine(
                                    new Pose(131.188, 58.218),
                                    new Pose(91.886, 86.869)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .addPath(
                            new BezierCurve(
                                    new Pose(91.886, 86.869),
                                    new Pose(66.285, 33.223),
                                    new Pose(105.993, 33.351)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .addPath(
                            new BezierLine(
                                    new Pose(105.993, 33.351),
                                    new Pose(132.763, 34.135)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .addPath(
                            new BezierLine(
                                    new Pose(132.763, 34.135),
                                    new Pose(92.297, 87.985)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .addPath(
                            new BezierLine(
                                    new Pose(92.297, 87.985),
                                    new Pose(84.081, 100.837)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
                    .build();
            return MainChain;
        }

        @Override
        public void init(){
                follower = Constants.createFollower(hardwareMap);
                follower.setPose(new Pose(124.255, 120.461, Math.toRadians(37)));
        }

        public void start() {
                follower.followPath(Paths(follower));
        }

        @Override
        public void loop() {
                follower.update();
        }
}