package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.Pose2d;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.pedroCommand.HoldPointCommand;
import com.seattlesolvers.solverslib.pedroCommand.TurnCommand;
import com.seattlesolvers.solverslib.pedroCommand.TurnToCommand;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pose.PathAutonomousGoalPedro;
import org.firstinspires.ftc.teamcode.vision.Pattern;

@Autonomous
public class AutonomousGoalStartPedro extends CommandOpMode {
    Follower follower;
    PathAutonomousGoalPedro mPath;
    Robot mRobot = new Robot();


    Pose pose = new Pose(
            72, 72, 90
    );

    PathChain pathChain;

    @Override
    public void initialize() {
        super.reset();

        follower = Constants.createFollower(hardwareMap);

        pathChain = follower.pathBuilder()
                .addPath(new BezierLine(
                        mPath.start(),
                        mPath.shootingFar())
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .addPath(new BezierLine(
                        mPath.shootingFar(),
                        mPath.startIntake(Pattern.PPG))
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .addPath(new BezierLine(
                        mPath.startIntake(Pattern.PPG),
                        mPath.endIntake(Pattern.PPG))
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .addPath(new BezierLine(
                        mPath.endIntake(Pattern.PPG),
                        mPath.shootingFar())
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .addPath(new BezierLine(
                        mPath.shootingFar(),
                        mPath.startIntake(Pattern.PGP))
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .addPath(new BezierLine(
                        mPath.startIntake(Pattern.PGP),
                        mPath.endIntake(Pattern.PGP))
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .addPath(new BezierLine(
                        mPath.endIntake(Pattern.PGP),
                        mPath.shootingFar())
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .addPath(new BezierLine(
                        mPath.shootingFar(),
                        mPath.startIntake(Pattern.GPP))
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .addPath(new BezierLine(
                        mPath.startIntake(Pattern.GPP),
                        mPath.endIntake(Pattern.GPP))
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .addPath(new BezierLine(
                        mPath.startIntake(Pattern.GPP),
                        mPath.endIntake(Pattern.GPP))
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .addPath(new BezierLine(
                        mPath.endIntake(Pattern.PGP),
                        mPath.shootingFar())
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .addPath(new BezierLine(
                        mPath.shootingFar(),
                        mPath.leave())
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))




                .build();


        schedule(
                // Updates follower to follow path

                // FollowPathCommand
                new FollowPathCommand(follower, pathChain, true, 1.0).setGlobalMaxPower(1.0)
        );

    }

    @Override
    public void run() {
        super.run();
    }
}