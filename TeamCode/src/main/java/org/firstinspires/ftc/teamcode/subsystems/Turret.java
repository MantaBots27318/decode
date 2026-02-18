/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   Intake brushes subsystem
   ------------------------------------------------------- */
package org.firstinspires.ftc.teamcode.subsystems;

/* Qualcomm includes */

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;


import org.firstinspires.ftc.teamcode.components.MotorComponent;
import org.firstinspires.ftc.teamcode.components.MotorCoupled;
import org.firstinspires.ftc.teamcode.components.MotorMock;
import org.firstinspires.ftc.teamcode.components.MotorSingle;
import org.firstinspires.ftc.teamcode.components.ServoComponent;
import org.firstinspires.ftc.teamcode.components.ServoCoupled;
import org.firstinspires.ftc.teamcode.components.ServoMock;
import org.firstinspires.ftc.teamcode.components.ServoSingle;
import org.firstinspires.ftc.teamcode.configurations.ConfLimelight;
import org.firstinspires.ftc.teamcode.configurations.ConfMotor;
import org.firstinspires.ftc.teamcode.configurations.ConfServo;
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.pose.Path;
import org.firstinspires.ftc.teamcode.utils.Logger;
import org.firstinspires.ftc.teamcode.utils.PositionMath;
import org.firstinspires.ftc.teamcode.utils.SmartTimer;
import org.firstinspires.ftc.teamcode.vision.Vision;

public class Turret {

    static final double     sRotationAmplitude = 4 * Math.PI;
    static final int        sResetTimeMs = 1000;

    Logger                  mLogger;
    boolean                 mReady;
    boolean                 mShallReset;
    SmartTimer              mTimer;

    double                  mDistanceCenterLimelight;
    Pose2d                  mCenterPositionFTC;
    Path                    mPath;     // True if component is able to fulfil its mission

    Vision                  mVision;
    Vector2d                mDirection;

    MotorComponent          mFlywheel;       // Motor rotating the wheels
    ServoComponent          mRotation;
    ServoComponent          mHood;

    // Initialize component from configuration
    public void setHW(Configuration config, HardwareMap hwm, Logger logger) {

        mLogger     = logger;
        mReady      = true;
        mShallReset = false;
        mTimer      = new SmartTimer(logger);

        String status = "";
        ConfLimelight limelight = Configuration.s_Current.getLimelight("limelight");
        if(limelight == null){ mReady = false; status += " CONF";}
        else {
            mVision         = new Vision(limelight, hwm, "vision", mLogger);
        }

        ConfMotor wheels = config.getMotor("transfer-wheels");
        if(wheels == null)  { mReady = false; status += " CONF";}
        else {
            mFlywheel = MotorComponent.factory(wheels, hwm, "outtake-wheels", logger);
            if (!mFlywheel.isReady()) { mReady = false; status += " HW";}
        }

        ConfServo rotation = config.getServo("turret-rotation");
        if(rotation == null)  { mReady = false; status += " CONF";}
        else {
            mRotation = ServoComponent.factory(rotation, hwm, "turret-rotation", logger);
            if (!mRotation.isReady()) { mReady = false; status += " HW";}
            else {
                mRotation.setPosition(this.initialize_rotation());
            }
        }

        ConfServo hood = config.getServo("turret-hood");
        if(hood == null)  { mReady = false; status += " CONF";}
        else {
            mHood = ServoComponent.factory(hood, hwm, "turret-hood", logger);
            if (!mHood.isReady()) { mReady = false; status += " HW";}
        }

        Pose2d radius = config.getPosition("limelight-rotation-radius");
        if(radius == null) { mReady = false; status += " CONF";}
        else {
            mDistanceCenterLimelight = radius.position.x;
        }
        mCenterPositionFTC = null;

        // Log status
        if (mReady) { logger.info("==>  TURRET : OK"); }
        else        { logger.warning("==>  TURRET : KO : " + status); }

    }

    public void close() {
        if(mReady) {
            mVision.close();
        }
    }

    public Pose2d getFTCPosition() {
        return mCenterPositionFTC;
    }

    public void loop(double velocityX, double velocityY, double deltaTime) {

        if(mReady) {
            Pose3D output = mVision.getPosition();
            if (output != null) {

                Pose2d limelightFTC = this.convertLimelightPoseToFTC(output);
                Pose2d limelightTurret = this.calculerPoseLimelightRobot(mRotation.getPosition(), mDistanceCenterLimelight);
                mCenterPositionFTC = PositionMath.getRobotPoseFromLimelight(limelightFTC, limelightTurret);
                double deltaAngle = this.angularError(mPath.qrcode(), mCenterPositionFTC, velocityX, velocityY, deltaTime);
                double servo_position = this.calculateServoPosition(deltaAngle, mRotation.getPosition(), mShallReset);
                mRotation.setPosition(servo_position);
            }

            if (!mTimer.isArmed()) { mShallReset = false; }
        }
    }

    public void reset() {
        mShallReset = true;
        mTimer.arm(sResetTimeMs);
    }

    private Pose2d convertLimelightPoseToFTC(Pose3D limelight){

        Pose2d pose = new Pose2d(
                -limelight.getPosition().x * Path.M_TO_INCHES,
                -limelight.getPosition().y * Path.M_TO_INCHES,
                (limelight.getOrientation().getYaw(AngleUnit.RADIANS) + 180) * Math.PI / 180);
        return pose;
    }

    private Pose2d calculerPoseLimelightRobot(double position_servo, double radius_turret){

        double angle = position_servo * sRotationAmplitude;
        while(angle < Math.PI) { angle += 2 * Math.PI; }
        while(angle > Math.PI) { angle -= 2 * Math.PI; }

        double yRobot = radius_turret * Math.cos(angle);
        double xRobot = radius_turret * Math.sin(angle);

        return new Pose2d(xRobot,yRobot,angle);
    }
    private double angularError(Pose2d target, Pose2d turret, double vx, double vy, double deltaTime){

        mLogger.info(String.format("==> PPT : X: %2.2f Y: %2.2f HD : %2.2f" , turret.position.x,turret.position.y,turret.heading.toDouble() / Math.PI * 180));

        Vector2d pos_ftc = new Vector2d(target.position.x - turret.position.x, target.position.y - turret.position.y);
        double length = pos_ftc.norm();
        double theta1 = Math.atan2(pos_ftc.y,pos_ftc.x);
        double theta2 = target.heading.toDouble() - theta1;

        mDirection = new Vector2d(length*Math.sin(theta2),length*Math.cos(theta2));
        double yaw = -Math.atan2(mDirection.x, mDirection.y);
        double error = turret.heading.toDouble() - yaw - target.heading.toDouble() ;

        mLogger.info(String.format("==> TRT ERROR: %2.2f ",error / Math.PI * 180));

        double rotation_speed = (pos_ftc.y * vx - pos_ftc.x * vy) / length / length;
        error += rotation_speed * deltaTime;

        mLogger.info(String.format("==> TRT ERROR + SPD: %2.2f ",error / Math.PI * 180));

        return error;

    }

    private double calculateServoPosition(double delta_angle, double current_position, boolean shall_reset){

        double result = -1;

        double angle = current_position * sRotationAmplitude;
        angle += delta_angle;
        result = angle / sRotationAmplitude;

        while(result > 1) { result -= 2 * Math.PI / sRotationAmplitude; }
        while(result < 0) { result += 2 * Math.PI / sRotationAmplitude; }

        if(shall_reset) {
            if(Math.abs(1 - result) < Math.abs(result - 2 * Math.PI / sRotationAmplitude)) {
                result -= 2 * Math.PI / sRotationAmplitude;
            }
            if(Math.abs(result) < Math.abs(1 - result - 2 * Math.PI / sRotationAmplitude)) {
                result += 2 * Math.PI / sRotationAmplitude;
            }
        }

        return result;

    }

    private double initialize_rotation() {
         return  0;
        ///  to be changed
    }

}
