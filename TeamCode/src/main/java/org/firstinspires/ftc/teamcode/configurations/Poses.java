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

    public static final double X_GPP_PATTERN_INIT_INCHES = 25;
    public static final double X_PGP_PATTERN_INIT_INCHES = 55;
    public static final double X_PPG_PATTERN_INIT_INCHES = 70;
    public static final double Y_PATTERN_INIT_INCHES_BLUE = 0;
    public static final double Y_PATTERN_INIT_INCHES_RED = 0;
    public static final double ANGLE_PATTERN_INIT_RADIANS_BLUE = Math.PI / 2;
    public static final double ANGLE_PATTERN_INIT_RADIANS_RED = -Math.PI / 2;

    public static final double X_GPP_PATTERN_FTC_INCHES = X_GPP_PATTERN_INIT_INCHES + X_INIT_FTC_INCHES;
    public static final double X_PGP_PATTERN_FTC_INCHES = X_PGP_PATTERN_INIT_INCHES + X_INIT_FTC_INCHES;
    public static final double X_PPG_PATTERN_FTC_INCHES = X_PPG_PATTERN_INIT_INCHES + X_INIT_FTC_INCHES;
    public static final double Y_PATTERN_FTC_INCHES_BLUE = 36;
    public static final double Y_PATTERN_FTC_INCHES_RED = -36;
    public static final double ANGLE_PATTERN_FTC_RADIANS_BLUE = ANGLE_PATTERN_INIT_RADIANS_BLUE;
    public static final double ANGLE_PATTERN_FTC_RADIANS_RED = ANGLE_PATTERN_INIT_RADIANS_RED;

    public static final double Y_DELTA_INTAKE_INCHES_BLUE = 40;
    public static final double Y_DELTA_INTAKE_INCHES_RED = -40;

    public static final double TGT_INTAKE_TO_CALIBRATION_INIT_RADIANS_BLUE = -Math.PI/2;
    public static final double TGT_INTAKE_TO_CALIBRATION_INIT_RADIANS_RED = Math.PI/2;

    public static final double TGT_INTAKE_TO_CALIBRATION_FTC_RADIANS_BLUE = TGT_INTAKE_TO_CALIBRATION_INIT_RADIANS_BLUE;
    public static final double TGT_INTAKE_TO_CALIBRATION_FTC_RADIANS_RED = TGT_INTAKE_TO_CALIBRATION_INIT_RADIANS_RED;

    public static final double X_CALIBRATION_INIT_INCHES = 74;
    public static final double Y_CALIBRATION_INIT_INCHES_BLUE = 10;
    public static final double Y_CALIBRATION_INIT_INCHES_RED = -10;
    public static final double ANGLE_CALIBRATION_INIT_RADIANS_RED = -Math.PI / 4;
    public static final double ANGLE_CALIBRATION_INIT_RADIANS_BLUE = Math.PI / 4;

    public static final double X_CALIBRATION_FTC_INCHES = X_CALIBRATION_INIT_INCHES + X_INIT_FTC_INCHES;
    public static final double Y_CALIBRATION_FTC_INCHES_BLUE = Y_CALIBRATION_INIT_INCHES_BLUE + Y_INIT_FTC_INCHES_BLUE;
    public static final double Y_CALIBRATION_FTC_INCHES_RED = Y_CALIBRATION_INIT_INCHES_RED + Y_INIT_FTC_INCHES_RED;
    public static final double ANGLE_CALIBRATION_FTC_RADIANS_RED = ANGLE_CALIBRATION_INIT_RADIANS_RED;
    public static final double ANGLE_CALIBRATION_FTC_RADIANS_BLUE = ANGLE_CALIBRATION_INIT_RADIANS_BLUE;

    public static final double X_SHOOTING_FTC_INCHES          = 24;
    public static final double Y_SHOOTING_FTC_INCHES_BLUE     = 24;
    public static final double ANGLE_SHOOTING_FTC_RADIANS_BLUE= Math.PI / 4;
    public static final double Y_SHOOTING_FTC_INCHES_RED      =-24;
    public static final double ANGLE_SHOOTING_FTC_RADIANS_RED = - Math.PI / 4;
    
    public static final double X_PARKING_GATE_ZONE_FTC_INCHES = 0;
    public static final double Y_PARKING_GATE_ZONE_FTC_INCHES_BLUE = 48;
    public static final double Y_PARKING_GATE_ZONE_FTC_INCHES_RED = -48;
    public static final double ANGLE_PARKING_GATE_ZONE_FTC_RADIANS_RED = Math.PI/2;
    public static final double ANGLE_PARKING_GATE_ZONE_FTC_RADIANS_BLUE = -Math.PI/2;

    public static final double X_PARKING_LAUNCH_ZONE_FTC_INCHES = 60;
    public static final double Y_PARKING_LAUNCH_ZONE_FTC_INCHES_BLUE = 30;
    public static final double Y_PARKING_LAUNCH_ZONE_FTC_INCHES_RED = -30;
    public static final double ANGLE_PARKING_LAUNCH_ZONE_FTC_RADIANS = -Math.PI;

    public static final double ANGLE_AUTO_TO_TELEOP_LAUNCH_ZONE_RADIANS_RED = - Math.PI/2;
    public static final double ANGLE_AUTO_TO_TELEOP_LAUNCH_ZONE_RADIANS_BLUE = Math.PI/2;
    public static final double ANGLE_AUTO_TO_TELEOP_GATE_ZONE_RADIANS_RED = - Math.PI/2;
    public static final double ANGLE_AUTO_TO_TELEOP_GATE_ZONE_RADIANS_BLUE = Math.PI/2;

    public static final double ANGLE_OBELISK_RADIANS_BLUE = -Math.PI/180 * 60;
    public static final double ANGLE_OBELISK_RADIANS_RED = Math.PI/180 * 60;

    public static final double X_CALIBRATION_FROM_GOAL_INCHES = -30*Math.sqrt(2);


    Telemetry       mLogger;

    Vector2d        mPositionInitFTCInches = new Vector2d(0,0);
    double          mAngleInitFTCRadians = 0;

    Vector2d        mPositionPatternInitInches = new Vector2d(0,0);
    double          mAnglePatternInitRadians = 0;
    Vector2d        mPositionPatternFTCInches = new Vector2d(0,0);
    double          mAnglePatternFTCRadians = 0;

    double          mYDeltaIntakeInches = 0;

    double          mTgtIntakeToCalibrationInitRadians = 0;

    double          mTgtIntakeToCalibrationFTCRadians = 0;

    Vector2d        mPositionCalibrationInitInches = new Vector2d(0,0);
    double          mAngleCalibrationInitRadians = 0;

    Vector2d        mPositionCalibrationFTCInches = new Vector2d(0,0);
    double          mAngleCalibrationFTCRadians = 0;

    Vector2d        mPositionShootingInches = new Vector2d(0,0);
    double          mAngleShootingRadians = 0;

    Vector2d        mPositionParkingFTCInches = new Vector2d(0,0);
    double          mAngleParkingFTCRadians = 0;

    double          mAngleAutoToTeleopRadians = 0;

    double          mCalibrationFromGoalInches = 0;

    double          mAngleObeliskFTCRadians = 0;

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
                mPositionPatternInitInches = new Vector2d(X_GPP_PATTERN_INIT_INCHES, Y_PATTERN_INIT_INCHES_RED);
                mPositionPatternFTCInches = new Vector2d(X_GPP_PATTERN_FTC_INCHES, Y_PATTERN_FTC_INCHES_RED);
            }
            if (pattern == Vision.Pattern.PGP) {
                mPositionPatternInitInches = new Vector2d(X_PGP_PATTERN_INIT_INCHES, Y_PATTERN_INIT_INCHES_RED);
                mPositionPatternFTCInches = new Vector2d(X_PGP_PATTERN_FTC_INCHES, Y_PATTERN_FTC_INCHES_RED);
            }
            if (pattern == Vision.Pattern.PPG) {
                mPositionPatternFTCInches = new Vector2d(X_GPP_PATTERN_FTC_INCHES, Y_PATTERN_FTC_INCHES_RED);
                mPositionPatternInitInches = new Vector2d(X_GPP_PATTERN_INIT_INCHES, Y_PATTERN_INIT_INCHES_RED);
            }

            mAnglePatternInitRadians = ANGLE_PATTERN_INIT_RADIANS_RED;
            mAnglePatternFTCRadians = ANGLE_PATTERN_FTC_RADIANS_RED;

            mYDeltaIntakeInches = Y_DELTA_INTAKE_INCHES_RED;

            mTgtIntakeToCalibrationInitRadians = TGT_INTAKE_TO_CALIBRATION_INIT_RADIANS_RED;
            mTgtIntakeToCalibrationFTCRadians = TGT_INTAKE_TO_CALIBRATION_FTC_RADIANS_RED;

            mPositionCalibrationInitInches = new Vector2d(X_CALIBRATION_INIT_INCHES,Y_CALIBRATION_INIT_INCHES_RED);
            mAngleCalibrationInitRadians = ANGLE_CALIBRATION_INIT_RADIANS_RED;

            mPositionCalibrationFTCInches = new Vector2d(X_CALIBRATION_FTC_INCHES,Y_CALIBRATION_FTC_INCHES_RED);
            mAngleCalibrationFTCRadians = ANGLE_CALIBRATION_FTC_RADIANS_RED;

            mPositionShootingInches = new Vector2d(X_SHOOTING_FTC_INCHES,Y_SHOOTING_FTC_INCHES_RED);
            mAngleShootingRadians = ANGLE_SHOOTING_FTC_RADIANS_RED;

            mAngleObeliskFTCRadians = ANGLE_OBELISK_RADIANS_RED ;

            if(ShallParkInLaunchZone){
                mPositionParkingFTCInches = new Vector2d(X_PARKING_LAUNCH_ZONE_FTC_INCHES, Y_PARKING_LAUNCH_ZONE_FTC_INCHES_RED);
                mAngleParkingFTCRadians = ANGLE_PARKING_LAUNCH_ZONE_FTC_RADIANS;
                mAngleAutoToTeleopRadians = ANGLE_AUTO_TO_TELEOP_LAUNCH_ZONE_RADIANS_RED;
            }
            else{
                mPositionParkingFTCInches = new Vector2d(X_PARKING_GATE_ZONE_FTC_INCHES, Y_PARKING_GATE_ZONE_FTC_INCHES_RED);
                mAngleParkingFTCRadians = ANGLE_PARKING_GATE_ZONE_FTC_RADIANS_RED;
                mAngleAutoToTeleopRadians = ANGLE_AUTO_TO_TELEOP_GATE_ZONE_RADIANS_RED;
            }

            mCalibrationFromGoalInches = X_CALIBRATION_FROM_GOAL_INCHES ;

        }

        if (alliance == Alliance.BLUE) {

            mPositionInitFTCInches = new Vector2d(X_INIT_FTC_INCHES, Y_INIT_FTC_INCHES_BLUE);
            mAngleInitFTCRadians = ANGLE_INIT_FTC_RADIANS;

            if (pattern == Vision.Pattern.GPP) {
                mPositionPatternInitInches = new Vector2d(X_GPP_PATTERN_INIT_INCHES, Y_PATTERN_INIT_INCHES_BLUE);
                mPositionPatternFTCInches = new Vector2d(X_GPP_PATTERN_FTC_INCHES, Y_PATTERN_FTC_INCHES_BLUE);
            }
            if (pattern == Vision.Pattern.PGP) {
                mPositionPatternInitInches = new Vector2d(X_PGP_PATTERN_INIT_INCHES, Y_PATTERN_INIT_INCHES_BLUE);
                mPositionPatternFTCInches = new Vector2d(X_PGP_PATTERN_FTC_INCHES, Y_PATTERN_FTC_INCHES_BLUE);
            }
            if (pattern == Vision.Pattern.PPG) {
                mPositionPatternInitInches = new Vector2d(X_PPG_PATTERN_INIT_INCHES, Y_PATTERN_INIT_INCHES_BLUE);
                mPositionPatternFTCInches = new Vector2d(X_PPG_PATTERN_FTC_INCHES, Y_PATTERN_FTC_INCHES_BLUE);
            }

            mAnglePatternInitRadians = ANGLE_PATTERN_INIT_RADIANS_BLUE;
            mAnglePatternFTCRadians = ANGLE_PATTERN_FTC_RADIANS_BLUE;

            mYDeltaIntakeInches = Y_DELTA_INTAKE_INCHES_BLUE;

            mTgtIntakeToCalibrationInitRadians = TGT_INTAKE_TO_CALIBRATION_INIT_RADIANS_BLUE;
            mTgtIntakeToCalibrationFTCRadians = TGT_INTAKE_TO_CALIBRATION_FTC_RADIANS_BLUE;

            mPositionCalibrationInitInches = new Vector2d(X_CALIBRATION_INIT_INCHES,Y_CALIBRATION_INIT_INCHES_BLUE);
            mAngleCalibrationInitRadians = ANGLE_CALIBRATION_INIT_RADIANS_BLUE;

            mPositionCalibrationFTCInches = new Vector2d(X_CALIBRATION_FTC_INCHES,Y_CALIBRATION_FTC_INCHES_BLUE);
            mAngleCalibrationFTCRadians = ANGLE_CALIBRATION_FTC_RADIANS_BLUE;

            mPositionShootingInches = new Vector2d(X_SHOOTING_FTC_INCHES,Y_SHOOTING_FTC_INCHES_BLUE);
            mAngleShootingRadians = ANGLE_SHOOTING_FTC_RADIANS_BLUE;

            mAngleObeliskFTCRadians = ANGLE_OBELISK_RADIANS_BLUE ;

            if(ShallParkInLaunchZone){
                mPositionParkingFTCInches = new Vector2d(X_PARKING_LAUNCH_ZONE_FTC_INCHES, Y_PARKING_LAUNCH_ZONE_FTC_INCHES_BLUE);
                mAngleParkingFTCRadians = ANGLE_PARKING_LAUNCH_ZONE_FTC_RADIANS;
                mAngleAutoToTeleopRadians = ANGLE_AUTO_TO_TELEOP_LAUNCH_ZONE_RADIANS_BLUE;
            }
            else{
                mPositionParkingFTCInches = new Vector2d(X_PARKING_GATE_ZONE_FTC_INCHES, Y_PARKING_GATE_ZONE_FTC_INCHES_BLUE);
                mAngleParkingFTCRadians = ANGLE_PARKING_GATE_ZONE_FTC_RADIANS_BLUE;
                mAngleAutoToTeleopRadians = ANGLE_AUTO_TO_TELEOP_GATE_ZONE_RADIANS_BLUE;
            }

            mCalibrationFromGoalInches = X_CALIBRATION_FROM_GOAL_INCHES ;

        }
    }

    public Vector2d posInitFTCInches()           { return mPositionInitFTCInches; }
    public double   hInitFTCInches()             { return mAngleInitFTCRadians; }

    public Vector2d posPatternInitInches()       { return mPositionPatternInitInches; }
    public double   hPatternInitRadians()        { return mAnglePatternInitRadians; }

    public Vector2d posPatternFTCInches()        { return mPositionPatternFTCInches; }
    public double   hPatternFTCRadians()         { return mAnglePatternFTCRadians; }

    public Vector2d posCalibrationInitInches()   { return mPositionCalibrationInitInches; }
    public double   hCalibrationInitRadians()    { return mAngleCalibrationInitRadians; }

    public Vector2d posCalibrationFTCInches()   { return mPositionCalibrationFTCInches; }
    public double   hCalibrationFTCRadians()    { return mAngleCalibrationFTCRadians; }

    public Vector2d posShootingFTCInches()       { return mPositionShootingInches; }
    public double   hShootingFTCRadians()        { return mAngleShootingRadians; }

    public double   yDeltaIntakeInches()         { return mYDeltaIntakeInches; }

    public double   tgtIntakeToCalibrationInitRadians() {return mTgtIntakeToCalibrationInitRadians;}

    public double   tgtIntakeToCalibrationFTCRadians() {return mTgtIntakeToCalibrationFTCRadians;}

    public Vector2d posParkingFTCInches()        { return mPositionParkingFTCInches;}
    public double   hParkingFTCRadians()         { return mAngleParkingFTCRadians;}

    public double   hAutoToTeleopRadians()       { return mAngleAutoToTeleopRadians; }

    public double   xCalibrationFromGoal ()         { return mCalibrationFromGoalInches ;}

    public double   hObeliskFTCRadians ()        {return  mAngleObeliskFTCRadians ;}

    public void log() {
        mLogger.addLine("PATTERN INIT X : " + mPositionPatternInitInches.x + " Y: " + mPositionPatternInitInches.y + " H: " + mAnglePatternInitRadians);
        mLogger.addLine("PATTERN FTC X : " + mPositionPatternFTCInches.x + " Y: " + mPositionPatternFTCInches.y + " H: " + mAnglePatternFTCRadians);
        mLogger.addLine("INTAKE DELTA Y : " + mYDeltaIntakeInches);
        mLogger.addLine("TGT INTAKE TO CALIBRATION INIT : " + mTgtIntakeToCalibrationInitRadians);
        mLogger.addLine("CALIBRATION INIT X: " + mPositionCalibrationInitInches.x + " Y: " + mPositionCalibrationInitInches.y + " H: " + mAngleCalibrationInitRadians);
        mLogger.addLine("CALIBRATION FTC X: " + mPositionCalibrationFTCInches.x + " Y: " + mPositionCalibrationFTCInches.y + " H: " + mAngleCalibrationFTCRadians);
        mLogger.addLine("SHOOTING: " + mPositionShootingInches.x + " Y: " + mPositionShootingInches.y + " H: " + mAngleShootingRadians);
        mLogger.addLine("LEAVE: " + mPositionParkingFTCInches.x + " Y: " + mPositionParkingFTCInches.y + " H: " + mAngleParkingFTCRadians);
        mLogger.addLine("AUTO TO TELEOP: " + mAngleAutoToTeleopRadians);
    }

}