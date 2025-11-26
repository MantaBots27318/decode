package org.firstinspires.ftc.teamcode.configurations;



import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.vision.Vision;

public class Poses {

    public static final double FIELD_SIZE_INCHES              = 12 * 12;
    public static final double M_TO_INCHES                    = 39.37;

    public static final double X_INIT_FTC_INCHES = -FIELD_SIZE_INCHES / 2 + 9;
    public static final double Y_INIT_FTC_INCHES_BLUE = 10;
    public static final double Y_INIT_FTC_INCHES_RED = -10;
    public static final double ANGLE_INIT_FTC_RADIANS = 0;

    public static final double Y_BEFORE_PATTERN_INIT_INCHES_BLUE = -10;
    public static final double Y_BEFORE_PATTERN_INIT_INCHES_RED = 10;
    public static final double ANGLE_BEFORE_PATTERN_INIT_RADIANS_BLUE = -Math.PI / 8;
    public static final double ANGLE_BEFORE_PATTERN_INIT_RADIANS_RED = Math.PI / 8;

    public static final double X_GPP_PATTERN_INIT_INCHES = 25;
    public static final double X_PGP_PATTERN_INIT_INCHES = 55;
    public static final double X_PPG_PATTERN_INIT_INCHES = 70;
    public static final double Y_PATTERN_INIT_INCHES_BLUE = 0;
    public static final double Y_PATTERN_INIT_INCHES_RED = 0;
    public static final double ANGLE_PATTERN_INIT_RADIANS_BLUE = Math.PI / 2;
    public static final double ANGLE_PATTERN_INIT_RADIANS_RED = -Math.PI / 2;

    public static final double Y_DELTA_INTAKE_INCHES_BLUE = 40;
    public static final double Y_DELTA_INTAKE_INCHES_RED = -40;

    public static final double TGT_INTAKE_TO_CALIBRATION_INIT_RADIANS_BLUE = -Math.PI/2;
    public static final double TGT_INTAKE_TO_CALIBRATION_INIT_RADIANS_RED = Math.PI/2;

    public static final double X_CALIBRATION_INIT_INCHES = 74;
    public static final double Y_CALIBRATION_INIT_INCHES_BLUE = 0;
    public static final double Y_CALIBRATION_INIT_INCHES_RED = -0;
    public static final double ANGLE_CALIBRATION_INIT_RADIANS_RED = -Math.PI / 4;
    public static final double ANGLE_CALIBRATION_INIT_RADIANS_BLUE = Math.PI / 4;

    public static final double X_SHOOTING_FTC_INCHES          = 24;
    public static final double Y_SHOOTING_FTC_INCHES_BLUE     = 24;
    public static final double ANGLE_SHOOTING_FTC_RADIANS_BLUE= Math.PI / 4;
    public static final double Y_SHOOTING_FTC_INCHES_RED      =-24;
    public static final double ANGLE_SHOOTING_FTC_RADIANS_RED = - Math.PI / 4;
    
    public static final double X_PARKING_GATE_ZONE = 0;
    public static final double Y_PARKING_GATE_ZONE_BLUE = 48;
    public static final double Y_PARKING_GATE_ZONE_RED = -48;
    public static final double ANGLE_PARKING_GATE_ZONE_RED_RADIANS = Math.PI/2;
    public static final double ANGLE_PARKING_GATE_ZONE_BLUE_RADIANS = -Math.PI/2;

    public static final double X_PARKING_LAUNCH_ZONE = 60;
    public static final double Y_PARKING_LAUNCH_ZONE_BLUE = 24;
    public static final double Y_PARKING_LAUNCH_ZONE_RED = -24;
    public static final double ANGLE_PARKING_LAUNCH_ZONE_RADIANS = -Math.PI;

    public static final double ANGLE_AUTO_TO_TELEOP_RADIANS_RED = -Math.PI / 4;
    public static final double ANGLE_AUTO_TO_TELEOP_RADIANS_BLUE = Math.PI / 4;

    public static final double X_SHOOTING_POSITION_FROM_GOAL_INCHES = -30*Math.sqrt(2);

    public static final double X_LEAVE_POSITION_FROM_GOAL_INCHES_BLUE = 50;
    public static final double Y_LEAVE_POSITION_FROM_GOAL_INCHES_BLUE = 20;


    public static final double X_LEAVE_POSITION_FROM_GOAL_INCHES_RED = 50;
    public static final double Y_LEAVE_POSITION_FROM_GOAL_INCHES_RED = -20;

    public static final double ANGLE_LEAVE_POSITION_FROM_GOAL_INCHES = Math.PI ;


    Telemetry       mLogger;

    Vector2d        mPositionInitFTCInches = new Vector2d(0,0);
    double          mAngleInitFTCRadians = 0;

    Vector2d        mPositionBeforePatternInitInches = new Vector2d(0,0);
    double          mAngleBeforePatternInitRadians = 0;

    Vector2d        mPositionPatternInitInches = new Vector2d(0,0);
    double          mAnglePatternInitRadians = 0;

    double          mYDeltaIntakeInches = 0;

    double          mTgtIntakeToCalibrationInitRadians = 0;

    Vector2d        mPositionCalibrationInitInches = new Vector2d(0,0);
    double          mAngleCalibrationInitRadians = 0;

    Vector2d        mPositionShootingInches = new Vector2d(0,0);
    double          mAngleShootingRadians = 0;

    Vector2d        mPositionParkingFTCInches = new Vector2d(0,0);
    double          mAngleParkingFTCRadians = 0;

    double          mAngleAutoToTeleopRadians = 0;



    double          mShootingPositionFromGoalInches = 0;

    Vector2d        mLeavePositionFromGoalInches = new Vector2d(0,0);
    double          mAngleLeaveRadians = 0;

    public Poses(Telemetry logger) {
        mLogger = logger;
    }

    public void initialize(Alliance alliance, Vision.Pattern pattern, boolean ShallParkInLaunchZone) {

        if (alliance == Alliance.RED) {

            mPositionInitFTCInches = new Vector2d(X_INIT_FTC_INCHES, Y_INIT_FTC_INCHES_RED);
            mAngleInitFTCRadians = ANGLE_INIT_FTC_RADIANS;

            if (pattern == Vision.Pattern.GPP) {
                mPositionBeforePatternInitInches = new Vector2d(X_GPP_PATTERN_INIT_INCHES - 10, Y_BEFORE_PATTERN_INIT_INCHES_RED);
                mPositionPatternInitInches = new Vector2d(X_GPP_PATTERN_INIT_INCHES, Y_PATTERN_INIT_INCHES_RED);
            }
            if (pattern == Vision.Pattern.PGP) {
                mPositionBeforePatternInitInches = new Vector2d(X_PGP_PATTERN_INIT_INCHES - 10, Y_BEFORE_PATTERN_INIT_INCHES_RED);
                mPositionPatternInitInches = new Vector2d(X_PGP_PATTERN_INIT_INCHES, Y_PATTERN_INIT_INCHES_RED);
            }
            if (pattern == Vision.Pattern.PPG) {
                mPositionBeforePatternInitInches = new Vector2d(X_PPG_PATTERN_INIT_INCHES - 10, Y_BEFORE_PATTERN_INIT_INCHES_RED);
                mPositionPatternInitInches = new Vector2d(X_PPG_PATTERN_INIT_INCHES, Y_PATTERN_INIT_INCHES_RED);
            }

            mAngleBeforePatternInitRadians = ANGLE_BEFORE_PATTERN_INIT_RADIANS_RED;
            mAnglePatternInitRadians = ANGLE_PATTERN_INIT_RADIANS_RED;


            mYDeltaIntakeInches = Y_DELTA_INTAKE_INCHES_RED;

            mTgtIntakeToCalibrationInitRadians = TGT_INTAKE_TO_CALIBRATION_INIT_RADIANS_RED;

            mPositionCalibrationInitInches = new Vector2d(X_CALIBRATION_INIT_INCHES,Y_CALIBRATION_INIT_INCHES_RED);
            mAngleCalibrationInitRadians = ANGLE_CALIBRATION_INIT_RADIANS_RED;

            mPositionShootingInches = new Vector2d(X_SHOOTING_FTC_INCHES,Y_SHOOTING_FTC_INCHES_RED);
            mAngleShootingRadians = ANGLE_SHOOTING_FTC_RADIANS_RED;



            if(ShallParkInLaunchZone){
                mPositionParkingFTCInches = new Vector2d(X_PARKING_LAUNCH_ZONE, Y_PARKING_LAUNCH_ZONE_RED);
                mAngleParkingFTCRadians = ANGLE_PARKING_LAUNCH_ZONE_RADIANS;
            }
            else{
                mPositionParkingFTCInches = new Vector2d(X_PARKING_GATE_ZONE, Y_PARKING_GATE_ZONE_RED);
                mAngleParkingFTCRadians = ANGLE_PARKING_GATE_ZONE_RED_RADIANS;
            }

            
            mAngleAutoToTeleopRadians = ANGLE_AUTO_TO_TELEOP_RADIANS_RED;

            mShootingPositionFromGoalInches = X_SHOOTING_POSITION_FROM_GOAL_INCHES ;
            mLeavePositionFromGoalInches = new Vector2d(X_LEAVE_POSITION_FROM_GOAL_INCHES_RED, Y_LEAVE_POSITION_FROM_GOAL_INCHES_RED );
            mAngleLeaveRadians = ANGLE_LEAVE_POSITION_FROM_GOAL_INCHES ;

        }

        if (alliance == Alliance.BLUE) {

            mPositionInitFTCInches = new Vector2d(X_INIT_FTC_INCHES, Y_INIT_FTC_INCHES_BLUE);
            mAngleInitFTCRadians = ANGLE_INIT_FTC_RADIANS;

            if (pattern == Vision.Pattern.GPP) {
                mPositionBeforePatternInitInches = new Vector2d(X_GPP_PATTERN_INIT_INCHES - 10, Y_BEFORE_PATTERN_INIT_INCHES_BLUE);
                mPositionPatternInitInches = new Vector2d(X_GPP_PATTERN_INIT_INCHES, Y_PATTERN_INIT_INCHES_BLUE);
            }
            if (pattern == Vision.Pattern.PGP) {
                mPositionBeforePatternInitInches = new Vector2d(X_PGP_PATTERN_INIT_INCHES - 10, Y_BEFORE_PATTERN_INIT_INCHES_BLUE);
                mPositionPatternInitInches = new Vector2d(X_PGP_PATTERN_INIT_INCHES, Y_PATTERN_INIT_INCHES_BLUE);
            }
            if (pattern == Vision.Pattern.PPG) {
                mPositionBeforePatternInitInches = new Vector2d(X_PPG_PATTERN_INIT_INCHES - 10, Y_BEFORE_PATTERN_INIT_INCHES_BLUE);
                mPositionPatternInitInches = new Vector2d(X_PPG_PATTERN_INIT_INCHES, Y_PATTERN_INIT_INCHES_BLUE);
            }

            mAngleBeforePatternInitRadians = ANGLE_BEFORE_PATTERN_INIT_RADIANS_BLUE;
            mAnglePatternInitRadians = ANGLE_PATTERN_INIT_RADIANS_BLUE;

            mYDeltaIntakeInches = Y_DELTA_INTAKE_INCHES_BLUE;

            mTgtIntakeToCalibrationInitRadians = TGT_INTAKE_TO_CALIBRATION_INIT_RADIANS_BLUE;

            mPositionCalibrationInitInches = new Vector2d(X_CALIBRATION_INIT_INCHES,Y_CALIBRATION_INIT_INCHES_BLUE);
            mAngleCalibrationInitRadians = ANGLE_CALIBRATION_INIT_RADIANS_BLUE;

            mPositionShootingInches = new Vector2d(X_SHOOTING_FTC_INCHES,Y_SHOOTING_FTC_INCHES_BLUE);
            mAngleShootingRadians = ANGLE_SHOOTING_FTC_RADIANS_BLUE;

            if(ShallParkInLaunchZone){
                mPositionParkingFTCInches = new Vector2d(X_PARKING_LAUNCH_ZONE, Y_PARKING_LAUNCH_ZONE_BLUE);
                mAngleParkingFTCRadians = ANGLE_PARKING_LAUNCH_ZONE_RADIANS;
            }
            else{
                mPositionParkingFTCInches = new Vector2d(X_PARKING_GATE_ZONE, Y_PARKING_GATE_ZONE_BLUE);
                mAngleParkingFTCRadians = ANGLE_PARKING_GATE_ZONE_BLUE_RADIANS;
            }

            mAngleAutoToTeleopRadians = ANGLE_AUTO_TO_TELEOP_RADIANS_BLUE;

            mShootingPositionFromGoalInches = X_SHOOTING_POSITION_FROM_GOAL_INCHES ;

            mLeavePositionFromGoalInches = new Vector2d(X_LEAVE_POSITION_FROM_GOAL_INCHES_BLUE, Y_LEAVE_POSITION_FROM_GOAL_INCHES_BLUE );
            mAngleLeaveRadians = ANGLE_LEAVE_POSITION_FROM_GOAL_INCHES ;

        }
    }

    public Vector2d posInitFTCInches()           { return mPositionInitFTCInches; }
    public double   hInitFTCInches()             { return mAngleInitFTCRadians; }


    public Vector2d posBeforePatternInitInches() { return mPositionBeforePatternInitInches; }
    public double   hBeforePatternInitRadians()  { return mAngleBeforePatternInitRadians; }

    public Vector2d posPatternInitInches()       { return mPositionPatternInitInches; }
    public double   hPatternInitRadians()        { return mAnglePatternInitRadians; }

    public Vector2d posCalibrationInitInches()   { return mPositionCalibrationInitInches; }
    public double   hCalibrationInitRadians()    { return mAngleCalibrationInitRadians; }

    public Vector2d posShootingFTCInches()       { return mPositionShootingInches; }
    public double   hShootingFTCRadians()        { return mAngleShootingRadians; }

    public double   yDeltaIntakeInches()         { return mYDeltaIntakeInches; }

    public double   tgtIntakeToCalibrationInitRadians() {return mTgtIntakeToCalibrationInitRadians;}

    public Vector2d posParkingFTCInches(){ return mPositionParkingFTCInches;}
    public double hParkingFTCRadians(){return mAngleParkingFTCRadians;}

    public double   hAutoToTeleopRadians()       { return mAngleAutoToTeleopRadians; }

    public double xShootingFromGoal ()             {return mShootingPositionFromGoalInches ;}

    public Vector2d posLeaveGoalInches()            {return mLeavePositionFromGoalInches ;}
    public double hLeaveGoalRadians ()              {return mAngleLeaveRadians ;}

    public void selectDistances(Vision.Pattern pattern) {


    }

    public void log() {
        mLogger.addLine("BEFORE PATTERN X: " + mPositionBeforePatternInitInches.x + " Y: " + mPositionBeforePatternInitInches.y + " H: " + mAngleBeforePatternInitRadians);
        mLogger.addLine("PATTERN X : " + mPositionPatternInitInches.x + " Y: " + mPositionPatternInitInches.y + " H: " + mAnglePatternInitRadians);
        mLogger.addLine("INTAKE DELTA Y : " + mYDeltaIntakeInches);
        mLogger.addLine("TGT INTAKE TO CALIBRATION : " + mTgtIntakeToCalibrationInitRadians);
        mLogger.addLine("CALIBRATION X: " + mPositionCalibrationInitInches.x + " Y: " + mPositionCalibrationInitInches.y + " H: " + mAngleCalibrationInitRadians);
        mLogger.addLine("SHOOTING: " + mPositionShootingInches.x + " Y: " + mPositionShootingInches.y + " H: " + mAngleShootingRadians);
        mLogger.addLine("LEAVE: " + mPositionParkingFTCInches.x + " Y: " + mPositionParkingFTCInches.y + " H: " + mAngleParkingFTCRadians);

    }

}