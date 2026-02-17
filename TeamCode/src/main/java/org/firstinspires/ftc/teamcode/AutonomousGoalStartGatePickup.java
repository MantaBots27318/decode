package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.components.Controller;
import org.firstinspires.ftc.teamcode.configurations.Alliance;
import org.firstinspires.ftc.teamcode.pose.PathAutonomousGoal;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.utils.Logger;
import org.firstinspires.ftc.teamcode.utils.SmartTimer;
import org.firstinspires.ftc.teamcode.vision.Pattern;
import org.firstinspires.ftc.teamcode.vision.Vision;

@Autonomous
public class AutonomousGoalStartGatePickup extends LinearOpMode {

    Vision mVision;
    MecanumDrive mDrive;
    Robot               mRobot;

    Pattern mPattern;
    Pattern             mTargetPattern;
    Pattern             mThirdPattern = Pattern.PPG;
    int                 mPatternShift = 0;
    Alliance mAlliance = Alliance.NONE;
    PathAutonomousGoal mPath;
    Pose2d mLimelightPositionInRR;
    double              mWaitingTime = 0.0;

    SmartTimer mTimer;

    Controller mGamepad1;
    Controller          mGamepad2;
    Camera mCamera;

    Logger mLogger;
    boolean             mShallGrabAnotherPattern;

    double              mShootVelocity = 2.7;


    public void runOpMode() throws InterruptedException {

    }


}
