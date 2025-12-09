/*
Copyright (c) 2024 Limelight Vision

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

// System includes
import java.util.ArrayList;
import java.util.List;

// ANDROIDX
import androidx.annotation.NonNull;

// QUALCOMM includes
import com.acmerobotics.roadrunner.Twist2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// ACME ROBOTICS includes
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;

// FTCController includes
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

// Local includes
import org.firstinspires.ftc.teamcode.configurations.Alliance;
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.path.Path;
import org.firstinspires.ftc.teamcode.path.PathAutonomousMiddle;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.SmartTimer;
import org.firstinspires.ftc.teamcode.utils.Logger;
import org.firstinspires.ftc.teamcode.vision.Vision;
import org.firstinspires.ftc.teamcode.components.Controller;
import org.firstinspires.ftc.teamcode.subsystems.camera.Camera;


@Autonomous
public class AutonomousMiddleStart extends LinearOpMode {

    Vision                  mVision;
    MecanumDrive            mDrive;
    Collecting              mCollecting;

    Vision.Pattern          mPattern;
    Vision.Pattern          mTargetPattern;
    int                     mPatternShift = 0;
    Alliance                mAlliance = Alliance.NONE;
    PathAutonomousMiddle    mPath;
    double                  mWaitingTime = 0.0;
    boolean                 mShallParkInLaunchZone = false;

    SmartTimer              mTimer;

    Controller              mGamepad1;
    Controller              mGamepad2;

    Camera                  mCamera;

    Logger                  mLogger;

    @Override
    public void runOpMode() throws InterruptedException {
        
        mCollecting     = new Collecting();

        mTimer          = new SmartTimer(telemetry);
        mLogger         = new Logger(telemetry, FtcDashboard.getInstance(),"autonomous-middle-start");

        mCamera         = new Camera();
        mCamera.setHW(Configuration.s_Current,hardwareMap,telemetry);

        mVision         = new Vision(Configuration.s_Current.getLimelight("limelight"), hardwareMap, "vision", telemetry);
        mVision.initialize();
        mPattern        = Vision.Pattern.PGP;
        mTargetPattern  = Vision.Pattern.PGP;

        mPath           = new PathAutonomousMiddle(mLogger);
        mDrive          = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        mGamepad1       = new Controller(gamepad1, mLogger);
        mGamepad2       = new Controller(gamepad2, mLogger);

        mCollecting     = new Collecting();
        mCollecting.setHW(Configuration.s_Current, hardwareMap, telemetry, mGamepad2);

        mCamera.setPosition(Camera.Position.TAG);

        while (opModeInInit()) {

            if (mGamepad1.buttons.dpad_right.pressedOnce())         {
                mAlliance = Alliance.RED;
                mPath.initialize(mAlliance,mTargetPattern, mShallParkInLaunchZone);
            }
            if (mGamepad1.buttons.dpad_left.pressedOnce())          {
                mAlliance = Alliance.BLUE;
                mPath.initialize(mAlliance, mTargetPattern, mShallParkInLaunchZone);
            }

            if (mGamepad1.buttons.dpad_up.pressedOnce())            { mWaitingTime += 1; mWaitingTime = Math.min(mWaitingTime,4);}
            if (mGamepad1.buttons.dpad_down.pressedOnce())          { mWaitingTime -= 1; mWaitingTime = Math.max(mWaitingTime,0); }

            if (mGamepad1.buttons.x.pressedOnce())  {
                mPatternShift -= 1;
                mPatternShift = Math.max(mPatternShift,0);
                if(mPattern == Vision.Pattern.GPP) { mTargetPattern = this.computePattern(mPattern,mPatternShift);}
                mPath.initialize(mAlliance, mTargetPattern,mShallParkInLaunchZone);
            }
            if (mGamepad1.buttons.b.pressedOnce()) {
                mPatternShift += 1;
                mPatternShift = Math.min(mPatternShift,3);
                if(mPattern == Vision.Pattern.GPP) { mTargetPattern = this.computePattern(mPattern,mPatternShift);}
                mPath.initialize(mAlliance, mTargetPattern,mShallParkInLaunchZone);
            }
            if(mGamepad1.buttons.y.pressedOnce()){
                mShallParkInLaunchZone = true;
            }
            if(mGamepad1.buttons.a.pressedOnce()){
                mShallParkInLaunchZone = false;
            }

            Vision.Pattern pattern = mVision.readPattern();
            if (pattern != Vision.Pattern.NONE) {
                mPattern = pattern;
                if(mPattern == Vision.Pattern.PPG) { mTargetPattern = Vision.Pattern.PGP; }
                if(mPattern == Vision.Pattern.PGP) { mTargetPattern = Vision.Pattern.PPG; }
                if(mPattern == Vision.Pattern.GPP) { mTargetPattern = this.computePattern(mPattern,mPatternShift);}
                mPath.initialize(mAlliance, mTargetPattern,mShallParkInLaunchZone);
            }

            mLogger.info("=========== MENU ============");
            mLogger.info("Choose Alliance: DPAD LEFT/RIGHT");
            mLogger.info("Choose Pattern Shift: X/B ");
            mLogger.info("Choose Waiting Time: DPAD UP/DOWN");
            mLogger.info("Choose Park Position: Y/A");

            mLogger.info("======= CONFIGURATION =======");
            mLogger.metric("==> PATTERN : " , mPattern.text());
            mLogger.metric("==> PATTERN SHIFT : " , "" + mPatternShift);
            mLogger.metric("==> PATTERN TARGET : " , mTargetPattern.text());
            mLogger.metric("==> ALLIANCE : ", mAlliance.name());
            mLogger.metric("==> WAITING TIME : ", mWaitingTime + " s");
            if (!mShallParkInLaunchZone) {
                mLogger.metric("==> PARKING POSITION","Gate Zone");
            }
            if (mShallParkInLaunchZone) {
                mLogger.metric("==> PARKING POSITION","Launch Zone");
            }
            mPath.log();

            mLogger.update();

        }

        mLogger.info("======= ACTIONS =======");
        mLogger.info("==> GO TO PATTERN");

        mLogger.update();

        mDrive = new MecanumDrive(hardwareMap,mPath.start());

        Action startIntakeAction = new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket p) {

                return mCollecting.start_intake();
            }
        };

        Action stopIntakeAction = new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket p) {

                return mCollecting.stop_intake();
            }
        };

        Action shakeAction = new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket p) {

                return mCollecting.shake();
            }
        };

        Action engageAction = new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket p) {

                return mCollecting.engage(0.80);
            }
        };

        Pose2d start = mPath.start();
        Pose2d pattern = mPath.pattern();
        Pose2d end_intake = mPath.endIntake();
        Pose2d back_intake = mPath.backIntake();
        Pose2d calibration = mPath.calibration();
        Pose2d shoot = mPath.shootingClose();
        Pose2d leave = mPath.parking();

        double distance_pattern = pattern.minus(start).line.norm();
        double distance_intake = end_intake.minus(pattern).line.norm();
        Actions.runBlocking(
            mDrive.actionBuilder(start)
                    .waitSeconds(mWaitingTime)
                    .afterDisp(0.4 * distance_pattern,startIntakeAction)
                    .setTangent(start.heading)
                    .splineToLinearHeading(pattern,start.heading)
                    .afterDisp(0.9 * distance_intake,stopIntakeAction)
                    .setTangent(pattern.heading.toDouble())
                    .splineToLinearHeading(end_intake,pattern.heading.toDouble(), new TranslationalVelConstraint(15), new ProfileAccelConstraint(-15,15))
                    .afterDisp(0.1 * distance_intake, shakeAction)
                    .setTangent(-end_intake.heading.toDouble())
                    .splineToLinearHeading(back_intake, -end_intake.heading.toDouble(), new TranslationalVelConstraint(100), new ProfileAccelConstraint(-50,50))
                    .afterDisp(0.1,engageAction)
                    .setTangent(mPath.tgtIntakeToCalibrationRadians())
                    .splineToLinearHeading(calibration,0, new TranslationalVelConstraint(50), new ProfileAccelConstraint(-30,30))
                    .build());


        updatePoseFromAprilTagIfVisible();

        mLogger.info("==> CALIBRATION");
        mLogger.info("REF POSE :" + mDrive.getPose());
        mLogger.info("==> GO TO SHOOTING");
        mLogger.update();

        Actions.runBlocking(
                mDrive.actionBuilder(mDrive.getPose())
                        .splineToLinearHeading(shoot,shoot.heading.toDouble())
                        .build());

        mCollecting.shoot4(0.75);

        Actions.runBlocking(
                mDrive.actionBuilder(shoot)
                        .setTangent(shoot.heading.toDouble() + Math.PI)
                        .splineToLinearHeading(leave, leave.heading.toDouble() + Math.PI)
                        .build());

        Configuration.s_Current.persist("heading", mPath.hAutoToTeleopRadians() + mDrive.getPose().heading.toDouble() - leave.heading.toDouble());
        Configuration.s_Current.persist("alliance",mAlliance.getValue());

        mVision.close();

    }

    void updatePoseFromAprilTagIfVisible() {
        // Gather April Tags
        Pose3D output = mVision.getPosition();
        mTimer.arm(100);

        while(output == null && mTimer.isArmed()) { output = mVision.getPosition(); }
        if (output != null) {

            Pose2d newReference = new Pose2d(
                    -output.getPosition().x * Path.M_TO_INCHES,
                    -output.getPosition().y * Path.M_TO_INCHES,
                    (output.getOrientation().getYaw() + 180) * Math.PI / 180);

            mDrive.updatePose(newReference);
        }

    }

    Vision.Pattern  computePattern(Vision.Pattern official, int shift) {

        Vision.Pattern result = Vision.Pattern.NONE;

        if(official != Vision.Pattern.NONE) {
            int target_identifier = (official.identifier() + shift) % 3;
            if (target_identifier == Vision.Pattern.GPP.identifier()) {
                result = Vision.Pattern.GPP;
            }
            if (target_identifier == Vision.Pattern.PGP.identifier()) {
                result = Vision.Pattern.PGP;
            }
            if (target_identifier == Vision.Pattern.PPG.identifier()) {
                result = Vision.Pattern.PPG;
            }
        }

        return result;
    }

}


