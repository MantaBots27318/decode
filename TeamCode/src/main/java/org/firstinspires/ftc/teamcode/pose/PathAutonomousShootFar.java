package org.firstinspires.ftc.teamcode.pose;

/* ACME includes */
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/* Project includes */
import org.firstinspires.ftc.teamcode.configurations.Alliance;

/* Vision includes */
import org.firstinspires.ftc.teamcode.vision.Pattern;

/* Utils includes */
import org.firstinspires.ftc.teamcode.utils.Logger;

public class PathAutonomousShootFar extends Path {
    private static final double X_START_INCHES = -57;
    private static final double Y_START_INCHES_BLUE = 19;
    private static final double Y_START_INCHES_RED = -19;
    private static final double ANGLE_START_RADIANS = 0;


    protected static final double Y_DELTA_INTAKE_INCHES_BLUE                     = 33;
    protected static final double Y_DELTA_INTAKE_INCHES_RED                      = -33;


    private static final double X_DELTA_GPP_PATTERN_INCHES_BLUE = 25;
    private static final double X_DELTA_PGP_PATTERN_INCHES_BLUE = 50;
    private static final double X_DELTA_PPG_PATTERN_INCHES_BLUE = 72;
    private static final double X_DELTA_GPP_PATTERN_INCHES_RED = 30;
    private static final double X_DELTA_PGP_PATTERN_INCHES_RED = 52;
    private static final double X_DELTA_PPG_PATTERN_INCHES_RED = 75;
    private static final double Y_DELTA_PATTERN_INCHES_BLUE = 0;
    private static final double Y_DELTA_PATTERN_INCHES_RED = 0;
    private static final double ANGLE_DELTA_PATTERN_RADIANS_BLUE = Math.PI / 2;
    private static final double ANGLE_DELTA_PATTERN_RADIANS_RED = -Math.PI / 2;

    private static final double X_SHOOTING_CALIBRATION_INCHES = 12;
    private static final double Y_SHOOTING_CALIBRATION_INCHES_BLUE = 12;
    private static final double Y_SHOOTING_CALIBRATION_INCHES_RED = -12;
    private static final double ANGLE_SHOOTING_CALIBRATION_RADIANS_RED = -0;
    private static final double ANGLE_SHOOTING_CALIBRATION_RADIANS_BLUE = 0;

    private static final double TGT_DELTA_INTAKE_TO_CALIBRATION_RADIANS_BLUE = -Math.PI/2;
    private static final double TGT_DELTA_INTAKE_TO_CALIBRATION_RADIANS_RED  = Math.PI/2;

    private static final double X_DELTA_CALIBRATION_INCHES                   = 74;
    private static final double Y_DELTA_CALIBRATION_INCHES_BLUE              = 10;
    private static final double Y_DELTA_CALIBRATION_INCHES_RED               = -10;
    private static final double ANGLE_DELTA_CALIBRATION_RADIANS_RED          = -Math.PI / 4;
    private static final double ANGLE_DELTA_CALIBRATION_RADIANS_BLUE         = Math.PI / 4;

    Pose2d  mStart                          = new Pose2d(0,0,0);
    Pose2d  mPattern                        = new Pose2d(0,0,0);
    Pose2d  mEndIntake                      = new Pose2d(0,0,0);
    Pose2d  mBackIntake                     = new Pose2d(0,0,0);
    Pose2d  mCalibration                    = new Pose2d(0,0,0);
    Pose2d mShoot = new Pose2d(0, 0, 0);

    double  mTgtIntakeToCalibrationRadians  = 0;

    public PathAutonomousShootFar(Logger logger) {
        super(logger);
    }


    public void initialize(Alliance alliance, Pattern pattern, boolean ShallParkInLaunchZone) {

        super.initialize(alliance, ShallParkInLaunchZone);

        if (alliance == Alliance.RED) {

            mStart = new Pose2d(X_START_INCHES, Y_START_INCHES_RED, ANGLE_START_RADIANS);
            mShoot  =  new Pose2d(X_SHOOTING_CALIBRATION_INCHES,Y_SHOOTING_CALIBRATION_INCHES_RED,ANGLE_SHOOTING_CALIBRATION_RADIANS_RED);
            if (pattern == Pattern.GPP) {
                mPattern = new Pose2d(
                        X_START_INCHES + X_DELTA_GPP_PATTERN_INCHES_RED,
                        Y_START_INCHES_RED + Y_DELTA_PATTERN_INCHES_RED,
                        ANGLE_START_RADIANS + ANGLE_DELTA_PATTERN_RADIANS_RED
                );
            }
            if (pattern == Pattern.PGP) {
                mPattern = new Pose2d(
                        X_START_INCHES + X_DELTA_PGP_PATTERN_INCHES_RED,
                        Y_START_INCHES_RED + Y_DELTA_PATTERN_INCHES_RED,
                        ANGLE_START_RADIANS + ANGLE_DELTA_PATTERN_RADIANS_RED);
            }
            if (pattern == Pattern.PPG) {
                mPattern = new Pose2d(
                        X_START_INCHES + X_DELTA_PPG_PATTERN_INCHES_RED,
                        Y_START_INCHES_RED + Y_DELTA_PATTERN_INCHES_RED,
                        ANGLE_START_RADIANS + ANGLE_DELTA_PATTERN_RADIANS_RED);
            }
            mEndIntake = new Pose2d(
                    mPattern.position.x,
                    mPattern.position.y + Y_DELTA_INTAKE_INCHES_RED,
                    mPattern.heading.toDouble());

            mBackIntake = new Pose2d(
                    mPattern.position.x,
                    mPattern.position.y + 0.3 * Y_DELTA_INTAKE_INCHES_RED,
                    mPattern.heading.toDouble());
            mCalibration = new Pose2d(
                    X_START_INCHES + X_DELTA_CALIBRATION_INCHES,
                    Y_START_INCHES_RED + Y_DELTA_CALIBRATION_INCHES_RED,
                    ANGLE_START_RADIANS + ANGLE_DELTA_CALIBRATION_RADIANS_RED);

            mTgtIntakeToCalibrationRadians = TGT_DELTA_INTAKE_TO_CALIBRATION_RADIANS_RED + ANGLE_START_RADIANS;

        }
        if (alliance == Alliance.BLUE) {
            mShoot  =  new Pose2d(X_SHOOTING_CALIBRATION_INCHES,Y_SHOOTING_CALIBRATION_INCHES_BLUE,ANGLE_SHOOTING_CALIBRATION_RADIANS_BLUE);
            mStart = new Pose2d(X_START_INCHES, Y_START_INCHES_BLUE, ANGLE_START_RADIANS);

            if (pattern == Pattern.GPP) {
                mPattern = new Pose2d(
                        X_START_INCHES + X_DELTA_GPP_PATTERN_INCHES_BLUE,
                        Y_START_INCHES_BLUE + Y_DELTA_PATTERN_INCHES_BLUE,
                        ANGLE_START_RADIANS + ANGLE_DELTA_PATTERN_RADIANS_BLUE);
            }
            if (pattern == Pattern.PGP) {
                mPattern = new Pose2d(
                        X_START_INCHES + X_DELTA_PGP_PATTERN_INCHES_BLUE,
                        Y_START_INCHES_BLUE + Y_DELTA_PATTERN_INCHES_BLUE,
                        ANGLE_START_RADIANS + ANGLE_DELTA_PATTERN_RADIANS_BLUE);
            }
            if (pattern == Pattern.PPG) {
                mPattern = new Pose2d(
                        X_START_INCHES + X_DELTA_PPG_PATTERN_INCHES_BLUE,
                        Y_START_INCHES_BLUE + Y_DELTA_PATTERN_INCHES_BLUE,
                        ANGLE_START_RADIANS + ANGLE_DELTA_PATTERN_RADIANS_BLUE);

            }
            mEndIntake = new Pose2d(
                    mPattern.position.x,
                    mPattern.position.y + Y_DELTA_INTAKE_INCHES_BLUE,
                    mPattern.heading.toDouble());

            mBackIntake = new Pose2d(
                    mPattern.position.x,
                    mPattern.position.y + 0.3 * Y_DELTA_INTAKE_INCHES_BLUE,
                    mPattern.heading.toDouble());
            mCalibration = new Pose2d(
                    X_START_INCHES + X_DELTA_CALIBRATION_INCHES,
                    Y_START_INCHES_BLUE + Y_DELTA_CALIBRATION_INCHES_BLUE,
                    ANGLE_START_RADIANS + ANGLE_DELTA_CALIBRATION_RADIANS_BLUE);

            mTgtIntakeToCalibrationRadians = TGT_DELTA_INTAKE_TO_CALIBRATION_RADIANS_BLUE + ANGLE_START_RADIANS;
        }
    }
    public Pose2d   start()                         { return mStart; }
    public Pose2d   pattern()                       { return mPattern; }
    public Pose2d   shoot()                         { return mShoot; }
    public Pose2d   endIntake()                     { return mEndIntake; }
    public Pose2d   backIntake()                    { return mBackIntake; }
    public double   tgtIntakeToCalibrationRadians() { return mTgtIntakeToCalibrationRadians;}
    public Pose2d   calibration()                   { return mCalibration; }
    }