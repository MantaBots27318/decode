/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   Intake brushes subsystem
   ------------------------------------------------------- */
package org.firstinspires.ftc.teamcode.subsystems;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

/* Acmerobotics includes */
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

/* Components includes */
import org.firstinspires.ftc.teamcode.components.MotorComponent;
import org.firstinspires.ftc.teamcode.components.EncoderComponent;
import org.firstinspires.ftc.teamcode.components.ServoComponent;

/* Configuration includes */
import org.firstinspires.ftc.teamcode.configurations.ConfLimelight;
import org.firstinspires.ftc.teamcode.configurations.ConfMotor;
import org.firstinspires.ftc.teamcode.configurations.ConfServo;
import org.firstinspires.ftc.teamcode.configurations.ConfEncoder;
import org.firstinspires.ftc.teamcode.configurations.Configuration;

/* Pose includes */
import org.firstinspires.ftc.teamcode.pose.Path;
import org.firstinspires.ftc.teamcode.pose.Posable;

/* Utils includes */
import org.firstinspires.ftc.teamcode.utils.ServoAbacus;
import org.firstinspires.ftc.teamcode.utils.Logger;
import org.firstinspires.ftc.teamcode.utils.SmartTimer;

/* Vision includes */
import org.firstinspires.ftc.teamcode.utils.VelocityAbacus;
import org.firstinspires.ftc.teamcode.vision.Vision;

import java.util.LinkedHashMap;
import java.util.Map;

public class Turret implements Posable{


    public enum Position {
        MIN,
        MAX
    }

    private static final Map<String, Position> sConfToPosition = Map.of(
            "min", Position.MIN,
            "max", Position.MAX
    );

    static final double     sRotationAmplitude = 4 * Math.PI;
    static final int        sRotationEncoderAmplitude = 20370;
    static final double     sMaxSpeed = 1950;
    static final double     sStartPosition = 0.5;

    Logger                  mLogger;
    boolean                 mReady;
    boolean                 mIsShooting;
    SmartTimer              mInitTimer;

    double                  mDistanceCenterLimelight;
    Pose2d                  mCenterPositionFTC;
    Pose2d                  mUpdatedPositionFTC;
    Path                    mPath;     // True if component is able to fulfil its mission

    Vision                  mVision;
    Vector2d                mDirection;

    MotorComponent          mFlywheel;       // Motor rotating the wheels
    ServoComponent          mRotation;
    ServoComponent          mHood;
    EncoderComponent        mRotationEncoder;

    double                  mInitialEncoderPosition;
    double                  mInitialServoPosition;
    Map<Position, Double>   mHoodPositions;   // Link between positions enumerated and servos positions
    Map<Position, Double>   mRotationPositions;   // Link between positions enumerated and servos positions

    boolean                mFirstLoop;

    // Initialize component from configuration
    public void setHW(Configuration config, HardwareMap hwm, Logger logger, Path path) {
        mLogger      = logger;
        mReady       = true;
        mFirstLoop   = true;
        mIsShooting  = false;
        mInitTimer   = new SmartTimer(logger);
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

        mRotationPositions   = new LinkedHashMap<>();
        ConfServo rotation = config.getServo("turret-rotation");
        if(rotation == null)  { mReady = false; status += " CONF ROT";}
        else {
            mRotation = ServoComponent.factory(rotation, hwm, "turret-rotation", logger);
            if (!mRotation.isReady()) { mReady = false; status += " HW ROT";}


            mRotationPositions.clear();
            Map<String, Double> confPosition = rotation.getPositions();
            for (Map.Entry<String, Double> pos : confPosition.entrySet()) {
                if(sConfToPosition.containsKey(pos.getKey())) {
                    mRotationPositions.put(sConfToPosition.get(pos.getKey()), pos.getValue());
                }  else {
                    mLogger.info("Found unmanaged turret rotation position : " + pos.getKey());
                }
            }

        }

        ConfEncoder turntable = config.getEncoder("turret-rotation");
        if(turntable == null)  { mReady = false; status += " CONF ROT ENC";}
        else {
            mRotationEncoder = EncoderComponent.factory(turntable, hwm, "turret-rotation", logger);
            if (!mRotationEncoder.isReady()) { mReady = false; status += " HW ROT EN";}
        }

        mHoodPositions   = new LinkedHashMap<>();
        ConfServo hood = config.getServo("turret-hood");
        if(hood == null)  { mReady = false; status += " CONF HD";}
        else {
            mHood = ServoComponent.factory(hood, hwm, "turret-hood", logger);
            if (!mHood.isReady()) { mReady = false; status += " HW HD";}

            mHoodPositions.clear();
            Map<String, Double> confPosition = hood.getPositions();
            for (Map.Entry<String, Double> pos : confPosition.entrySet()) {
                if(sConfToPosition.containsKey(pos.getKey())) {
                    mHoodPositions.put(sConfToPosition.get(pos.getKey()), pos.getValue());
                }  else {
                    mLogger.info("Found unmanaged hood position : " + pos.getKey());
                }
            }
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
            mRotation.setPosition(sStartPosition);
            mHood.setPosition(0.46);
            mInitTimer.arm(3000);
        }

    }

    public void close() {
        if(mReady) {
            mVision.close();
        }
    }

    public boolean isShooting() { return mIsShooting;}

    @Override
    public Pose2d getFTCPosition() {
        Pose2d result = mUpdatedPositionFTC;
        mUpdatedPositionFTC = null; /* Consumed */
        return result;
    }

    @Override
    public void setFTCPosition(Pose2d position) {
        if(mReady) {
            mCenterPositionFTC = position;
        }
    }

    public void loop(double velocityX, double velocityY, double deltaTime) {

        if(mReady && !mInitTimer.isArmed()) {

            if(mFirstLoop) {
                mInitialEncoderPosition = mRotationEncoder.getCurrentPosition();
                mLogger.metric("INITIAL ENCODER : ","" + mInitialEncoderPosition);
                mInitialServoPosition = mRotation.getPosition();
                mLogger.metric("INITIAL SERVO : ","" + mInitialServoPosition);
                mFirstLoop = false;
            }

            double delta_encoder = mRotationEncoder.getCurrentPosition() - mInitialEncoderPosition;
            mLogger.metric("DELTA ENCODER : ",""+ delta_encoder);
            double position = delta_encoder / sRotationEncoderAmplitude + mInitialServoPosition;
            mLogger.metric("CURRENT ROTATION SERVO POSITION : " , ""+ position);
            Pose2d limelightTurret = this.calculerPoseLimelightInTurretReference(position, mDistanceCenterLimelight);
            mLogger.metric("LIMELIGHT TURRET : " ,""+ limelightTurret.position + " " + limelightTurret.heading.toDouble() / Math.PI * 180);

            Pose3D output = mVision.getPosition();
            if (output != null) {
                Pose2d  limelightFTC = this.convertLimelightPoseToFTC(output);
                mLogger.metric("LIMELIGHT FTC : " , limelightFTC.position + " " + limelightFTC.heading.toDouble() / Math.PI * 180);
                mCenterPositionFTC = Posable.derivePose(limelightFTC, limelightTurret);
                mUpdatedPositionFTC = mCenterPositionFTC;
                mLogger.metric("TURRET FTC : " , mCenterPositionFTC.position + " " + mCenterPositionFTC.heading.toDouble() / Math.PI * 180);
            }

            Pose2d turret_orient = new Pose2d(mCenterPositionFTC.position.x, mCenterPositionFTC.position.y, mCenterPositionFTC.heading.toDouble() + limelightTurret.heading.toDouble());
            mLogger.metric("TURRET FTC : " , turret_orient.position + " " + turret_orient.heading.toDouble() / Math.PI * 180);
            double deltaAngle = this.angularError(mPath.target(), turret_orient, velocityX, velocityY, deltaTime);
            mLogger.metric("ANGULAR ERROR : " , "" + deltaAngle / Math.PI * 180);
            double rotation_servo_position = this.calculateRotationServoPosition(-deltaAngle, position);
            mLogger.metric("NEXT ROTATION SERVO POSITION : " , ""+rotation_servo_position);
            mRotation.setPosition(rotation_servo_position);
            double hood_servo_position = this.calculateHoodServoPosition(mPath.target(),mCenterPositionFTC);
            mLogger.metric("NEXT HOOD SERVO POSITION : " , ""+hood_servo_position);
            mHood.setPosition(hood_servo_position);
            double flywheel_speed = this.calculateFlywheelSpeed(mPath.target(),mCenterPositionFTC);
            mLogger.metric("FLYWHEEL SPEED : " , ""+flywheel_speed);
            if(mIsShooting) { mFlywheel.setVelocity(flywheel_speed); }

        }
    }

    public void start() {
        if(mReady) {
            double flywheel_speed = this.calculateFlywheelSpeed(mPath.target(),mCenterPositionFTC);
            mLogger.metric("FLYWHEEL SPEED : " , ""+flywheel_speed);
            mFlywheel.setVelocity(flywheel_speed);
            mIsShooting = true;
        }
    }
    
    public void stop() {
        if(mReady) {
            mFlywheel.setVelocity(0.0);
            mIsShooting = false;
        }
    }

    public void orient(double heading) {

        if(mReady && !(mInitTimer.isArmed())) {

            if(mFirstLoop ) {
                mInitialEncoderPosition = mRotationEncoder.getCurrentPosition();
                mLogger.info("INITIAL ENCODER : " + mInitialEncoderPosition);
                mInitialServoPosition = mRotation.getPosition();
                mLogger.info("INITIAL SERVO : " + mInitialServoPosition);
                mFirstLoop = false;
            }

            double delta_encoder = mRotationEncoder.getCurrentPosition() - mInitialEncoderPosition;
            mLogger.trace("DELTA ENCODER : " + delta_encoder);
            double position = delta_encoder / sRotationEncoderAmplitude + mInitialServoPosition;
            mLogger.trace("CURRENT ROTATION SERVO POSITION : " + position);
            Pose2d limelightTurret = this.calculerPoseLimelightInTurretReference(position, mDistanceCenterLimelight);
            mLogger.trace("LIMELIGHT TURRET : " + limelightTurret.position + " " + limelightTurret.heading.toDouble() / Math.PI * 180);

            Pose3D output = mVision.getPosition();
            if (output != null) {
                Pose2d  limelightFTC = this.convertLimelightPoseToFTC(output);
                mLogger.trace("LIMELIGHT FTC : " + limelightFTC.position + " " + limelightFTC.heading.toDouble() / Math.PI * 180);
                mCenterPositionFTC = Posable.derivePose(limelightFTC, limelightTurret);
                mLogger.trace("TURRET FTC : " + mCenterPositionFTC.position + " " + mCenterPositionFTC.heading.toDouble() / Math.PI * 180);
            }

            Pose2d turret_orient = new Pose2d(mCenterPositionFTC.position.x, mCenterPositionFTC.position.y, mCenterPositionFTC.heading.toDouble() + limelightTurret.heading.toDouble());
            mLogger.trace("TURRET FTC : " + turret_orient.position + " " + turret_orient.heading.toDouble() / Math.PI * 180);
            double deltaAngle = heading - turret_orient.heading.toDouble();
            mLogger.trace("ANGULAR ERROR : " + deltaAngle / Math.PI * 180);
            double rotation_servo_position = this.calculateRotationServoPosition(-deltaAngle, position);
            mLogger.trace("NEXT ROTATION SERVO POSITION : " + rotation_servo_position);
            mRotation.setPosition(rotation_servo_position);
            double hood_servo_position = this.calculateHoodServoPosition(mPath.target(),mCenterPositionFTC);
            mLogger.trace("NEXT HOOD SERVO POSITION : " + hood_servo_position);
            mHood.setPosition(hood_servo_position);
        }
    }

    private Pose2d convertLimelightPoseToFTC(Pose3D limelight){

        return new Pose2d(
                -limelight.getPosition().x * Path.M_TO_INCHES,
                -limelight.getPosition().y * Path.M_TO_INCHES,
                (limelight.getOrientation().getYaw() + 180) * Math.PI / 180);
    }

    private Pose2d calculerPoseLimelightInTurretReference(double position_servo, double radius_turret){
        
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
        double theta1 = Math.atan2(pos_ftc.y,pos_ftc.x);
        double theta2 = target.heading.toDouble() - theta1;

        mDirection = new Vector2d(length*Math.sin(theta2),length*Math.cos(theta2));
        double yaw = -Math.atan2(mDirection.x, mDirection.y);
        double error = turret.heading.toDouble() - yaw - target.heading.toDouble() ;

        mLogger.trace(String.format("==> TRT ERROR: %2.2f ",error / Math.PI * 180));

        double rotation_speed = (pos_ftc.y * vx - pos_ftc.x * vy) / length / length;
        error += rotation_speed * deltaTime;

        mLogger.trace(String.format("==> TRT ERROR + SPD: %2.2f ",error / Math.PI * 180));

        return error;

    }

    private double calculateRotationServoPosition(double delta_angle, double current_position){

        double result;

        double angle = current_position * sRotationAmplitude;
        angle += delta_angle;
        result = angle / sRotationAmplitude;
        //result = Math.round(result * 100) * 1.0/ 100;

        double result1 = result + 2 * Math.PI / sRotationAmplitude;
        double result2 = result - 2 * Math.PI / sRotationAmplitude;

        if((result1 >= mRotationPositions.get(Position.MIN)) &&  (result1 <= mRotationPositions.get(Position.MIN))){
            result = result1;
        }
        if((result2 >= mRotationPositions.get(Position.MIN)) &&  (result2 <= mRotationPositions.get(Position.MIN))){
            result = result2;
        }
        if(result > mRotationPositions.get(Position.MAX)) {
            result = mRotationPositions.get(Position.MAX);
        }
        if(result < mRotationPositions.get(Position.MIN)) {
            result = mRotationPositions.get(Position.MIN);
        }

        return result;

    }

    private double calculateHoodServoPosition(Pose2d target, Pose2d center) {

        double distance = Math.sqrt(
                (target.position.x - center.position.x) *
                (target.position.x - center.position.x) +
                (target.position.y - center.position.y) *
                (target.position.y - center.position.y));

        double result = ServoAbacus.getPosition(distance);
        if(result < mHoodPositions.get(Position.MIN)) { result = mHoodPositions.get(Position.MIN); }
        if(result > mHoodPositions.get(Position.MAX)) { result = mHoodPositions.get(Position.MAX); }

        return result;
    }

    private double calculateFlywheelSpeed(Pose2d target, Pose2d center) {

        double distance = Math.sqrt(
                (target.position.x - center.position.x) *
                        (target.position.x - center.position.x) +
                        (target.position.y - center.position.y) *
                                (target.position.y - center.position.y));

        double result = VelocityAbacus.getVelocity(distance);
        result *= sMaxSpeed;

        return result;
    }
}