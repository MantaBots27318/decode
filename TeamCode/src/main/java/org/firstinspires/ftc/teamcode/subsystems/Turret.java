/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   Intake brushes subsystem
   ------------------------------------------------------- */
package org.firstinspires.ftc.teamcode.subsystems;

/* Qualcomm includes */
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import org.firstinspires.ftc.teamcode.components.MotorComponent;
import org.firstinspires.ftc.teamcode.components.EncoderComponent;
import org.firstinspires.ftc.teamcode.components.ServoComponent;
import org.firstinspires.ftc.teamcode.configurations.ConfLimelight;
import org.firstinspires.ftc.teamcode.configurations.ConfMotor;
import org.firstinspires.ftc.teamcode.configurations.ConfServo;
import org.firstinspires.ftc.teamcode.configurations.ConfEncoder;
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.pose.Path;
import org.firstinspires.ftc.teamcode.utils.Logger;
import org.firstinspires.ftc.teamcode.utils.PositionMath;
import org.firstinspires.ftc.teamcode.utils.SmartTimer;
import org.firstinspires.ftc.teamcode.vision.Vision;

@Config
public class Turret {

    static final double     sRotationAmplitude = 4 * Math.PI;
    static final int        sResetTimeMs = 5000;
    static final int        sRotationEncoderAmplitude = 5917 - -16398;
    static final int        sProcessingPeriodMs = 200;
    static public double    sMaxSpeed = 1000;

    Logger                  mLogger;
    boolean                 mReady;
    boolean                 mShallReset;
    boolean                 mIsShooting;
    SmartTimer              mResetTimer;
    SmartTimer              mPeriodTimer;

    double                  mDistanceCenterLimelight;
    Pose2d                  mCenterPositionFTC;
    Path                    mPath;     // True if component is able to fulfil its mission

    Vision                  mVision;
    Vector2d                mDirection;

    MotorComponent          mFlywheel;       // Motor rotating the wheels
    ServoComponent          mRotation;
    ServoComponent          mHood;
    EncoderComponent        mRotationEncoder;

    double                  mInitialRotationPosition;

    // Initialize component from configuration
    public void setHW(Configuration config, HardwareMap hwm, Logger logger, Path path, Pose2d initial_position) {
        mLogger      = logger;
        mReady       = true;
        mIsShooting  = false;
        mShallReset  = false;
        mResetTimer  = new SmartTimer(logger);
        mPeriodTimer = new SmartTimer(logger);
        mPath        = path;

        String status = "";
        ConfLimelight limelight = Configuration.s_Current.getLimelight("limelight");
        if(limelight == null){ mReady = false; status += " CONF LL" ;}
        else {
            mVision         = new Vision(limelight, hwm, "vision", mLogger);
            mVision.initialize();
        }

        ConfMotor wheels = config.getMotor("outtake-wheels");
        if(wheels == null)  { mReady = false; status += " CONF FLYWHEEL";}
        else {
            mFlywheel = MotorComponent.factory(wheels, hwm, "outtake-wheels", logger);
            if (!mFlywheel.isReady()) { mReady = false; status += " HW FLYWHEEL";}
        }

        ConfServo rotation = config.getServo("turret-rotation");
        if(rotation == null)  { mReady = false; status += " CONF ROT";}
        else {
            mRotation = ServoComponent.factory(rotation, hwm, "turret-rotation", logger);
            if (!mRotation.isReady()) { mReady = false; status += " HW ROT";}
            else {
                mRotation.setPosition(0);
                //this.initialize_rotation(initial_position);
                mResetTimer.arm(sResetTimeMs);
                while(mResetTimer.isArmed()) { try { Thread.sleep(100); } catch(Exception e) {}}
            }
        }

        ConfEncoder turntable = config.getEncoder("turret-rotation");
        if(turntable == null)  { mReady = false; status += " CONF ROT ENC";}
        else {
            mRotationEncoder = EncoderComponent.factory(turntable, hwm, "turret-rotation", logger);
            if (!mRotationEncoder.isReady()) { mReady = false; status += " HW ROT EN";}
            else {
                mInitialRotationPosition = mRotationEncoder.getCurrentPosition();
                mLogger.info("initial encoder : " + mInitialRotationPosition);
            }
        }

        ConfServo hood = config.getServo("turret-hood");
        if(hood == null)  { mReady = false; status += " CONF HD";}
        else {
            mHood = ServoComponent.factory(hood, hwm, "turret-hood", logger);
            if (!mHood.isReady()) { mReady = false; status += " HW HD";}
        }

        Pose2d radius = config.getPosition("limelight-rotation-radius");
        if(radius == null) { mReady = false; status += " CONF RADIUS";}
        else {
            mDistanceCenterLimelight = radius.position.x;
        }
        mCenterPositionFTC = null;

        // Log status
        if (mReady) { logger.info("==>  TURRET : OK"); }
        else        { logger.warning("==>  TURRET : KO : " + status); }

        if(mReady) {
            //this.initialize_rotation(initial_position);
            //mResetTimer.arm(sResetTimeMs);
            //while(mResetTimer.isArmed()) { try { Thread.sleep(100); } catch(Exception e) {}}
        }

    }

    public void close() {
        if(mReady) {
            mVision.close();
        }
    }

    public boolean isShooting() { return mIsShooting;}

    public Pose2d getFTCPosition() {
        return mCenterPositionFTC;
    }

    public void loop(double velocityX, double velocityY, double deltaTime) {

        if(mReady && !mPeriodTimer.isArmed()) {

            Pose3D output = mVision.getPosition();
            if (output != null) {
                Pose2d limelightFTC = this.convertLimelightPoseToFTC(output);
                mLogger.info("LIMELIGHT FTC : " + limelightFTC.position + " " + limelightFTC.heading.toDouble() / Math.PI * 180);
                double delta_encoder = mRotationEncoder.getCurrentPosition() - mInitialRotationPosition;
                mLogger.info("DELTA ENCODER" + delta_encoder);
                double position = delta_encoder / sRotationEncoderAmplitude;
                Pose2d limelightTurret = this.calculerPoseLimelightRobot(position, mDistanceCenterLimelight);
                mLogger.info("LIMELIGHT TURRET : " + limelightTurret.position + " " + limelightTurret.heading.toDouble()/ Math.PI * 180);
                mCenterPositionFTC = PositionMath.getRobotPoseFromLimelight(limelightFTC, limelightTurret);
                mLogger.info("TURRET FTC : " + mCenterPositionFTC.position + " " + mCenterPositionFTC.heading.toDouble()/ Math.PI * 180);
                mCenterPositionFTC = new Pose2d(mCenterPositionFTC.position.x,mCenterPositionFTC.position.y,limelightFTC.heading.toDouble());
                mLogger.info("TURRET FTC : " + mCenterPositionFTC.position + " " + mCenterPositionFTC.heading.toDouble()/ Math.PI * 180);
                double deltaAngle = this.angularError(mPath.target(), mCenterPositionFTC, velocityX, velocityY, deltaTime);
                double servo_position = this.calculateServoPosition(-deltaAngle, position, mShallReset);
                mLogger.info("SERVO POSITION : " + servo_position);
                mRotation.setPosition(servo_position);
            }

            if (!mResetTimer.isArmed()) { mShallReset = false; }
            mPeriodTimer.arm(sProcessingPeriodMs);
        }
    }

    public void shoot() {
        if(mReady) {
            mFlywheel.setPower(1.0);
            mIsShooting = true;
        }
    }
    
    public void stop() {
        if(mReady) {
            mFlywheel.setPower(0.0);
            mIsShooting = false;
        }
    }

    public void reset() {
        mShallReset = true;
        mResetTimer.arm(sResetTimeMs);
    }

    private Pose2d convertLimelightPoseToFTC(Pose3D limelight){

        return new Pose2d(
                -limelight.getPosition().x * Path.M_TO_INCHES,
                -limelight.getPosition().y * Path.M_TO_INCHES,
                (limelight.getOrientation().getYaw() + 180) * Math.PI / 180);
    }

    private Pose2d calculerPoseLimelightRobot(double position_servo, double radius_turret){
        
        double angle = position_servo * sRotationAmplitude;
        while(angle < Math.PI) { angle += 2 * Math.PI; }
        while(angle > Math.PI) { angle -= 2 * Math.PI; }

        double xRobot = radius_turret * Math.cos(angle);
        double yRobot = radius_turret * Math.sin(angle);

        return new Pose2d(xRobot,yRobot,angle);
    }
    private double angularError(Pose2d target, Pose2d turret, double vx, double vy, double deltaTime){

        mLogger.info(String.format("==> TRT : X: %2.2f Y: %2.2f HD : %2.2f" , turret.position.x,turret.position.y,turret.heading.toDouble() / Math.PI * 180));
        mLogger.info(String.format("==> TGT : X: %2.2f Y: %2.2f HD : %2.2f" , target.position.x,target.position.y,target.heading.toDouble() / Math.PI * 180));

        Vector2d pos_ftc = new Vector2d(target.position.x - turret.position.x, target.position.y - turret.position.y);
        double length = pos_ftc.norm();
        mLogger.info(length+"length");
        double theta1 = Math.atan2(pos_ftc.y,pos_ftc.x);
        mLogger.info(theta1+"theta1");
        double theta2 = target.heading.toDouble() - theta1;
        mLogger.info(theta2+"theta");

        mDirection = new Vector2d(length*Math.sin(theta2),length*Math.cos(theta2));
        mLogger.info(mDirection+"direction");
        double yaw = -Math.atan2(mDirection.x, mDirection.y);
        mLogger.info(yaw+"yaw");
        double error = turret.heading.toDouble() - yaw - target.heading.toDouble() ;
        mLogger.info(error+"error");

        mLogger.info(String.format("==> TRT ERROR: %2.2f ",error / Math.PI * 180));

        double rotation_speed = (pos_ftc.y * vx - pos_ftc.x * vy) / length / length;
        error += rotation_speed * deltaTime;

        mLogger.info(String.format("==> TRT ERROR + SPD: %2.2f ",error / Math.PI * 180));

        return error;

    }

    private double calculateServoPosition(double delta_angle, double current_position, boolean shall_reset){

        double result;

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

    private void initialize_rotation(Pose2d initial){
        double deltaAngle = this.angularError(mPath.target(), initial, 0, 0, 0);
        double servo_position = this.calculateServoPosition(-deltaAngle, 0, false);
        mLogger.info("SERVO POSITION : " + servo_position);
        mRotation.setPosition(0);
    }

}