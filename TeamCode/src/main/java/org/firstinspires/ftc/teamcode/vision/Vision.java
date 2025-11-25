package org.firstinspires.ftc.teamcode.vision;

// Java includes
import java.util.ArrayList;
import java.util.List;

// Qualcomm includes
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

// Acmerobotics includes
import com.acmerobotics.dashboard.FtcDashboard;

// FTC Controller includes
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.configurations.ConfLimelight;

// Opencv includes
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.configurations.LastYear;
import org.opencv.core.Point;

public class Vision {

    public enum Pattern {

        GPP("GPP", 0),
        PPG("PPG", 1),
        PGP("PGP", 2),
        NONE("None", -1);

        private final String mText;
        private final int    mIdentifier;

        Pattern(String text, int identifier) {
            mText = text;
            mIdentifier = identifier;
        };

        public String text() { return mText; }
        public int    identifier() { return mIdentifier; }
    }

    private Limelight3A  mLimelight;
    private Telemetry    mTelemetry;
    private Calibration  mCalibration;

    private Integer      mAprilTagPipeline;
    private Integer      mDetectionPipeline;

    public Vision(ConfLimelight conf, HardwareMap hwMap, String name, Telemetry logger) {

        mTelemetry = logger;
        mTelemetry.addLine("Constructor");

        String camera_name = conf.getHw();
        mLimelight = hwMap.get(Limelight3A.class, camera_name);

        mAprilTagPipeline = conf.getPipeline("localizer");
        mDetectionPipeline = conf.getPipeline("balls-detector");

        mCalibration = new Calibration() ;
    }

    public void initialize() {
        mTelemetry.addLine("Initialize");

        mLimelight.start();
        mCalibration.warp();

    }

    public void close() {
        mLimelight.stop();
    }

    public Pattern readPattern() {

        Pattern pattern = Pattern.NONE;
        int apriltagId = 0;

        mLimelight.pipelineSwitch(mAprilTagPipeline);
        LLResult result = mLimelight.getLatestResult();

        if (result != null) {

            //telemetry.addData(result.isValid());
            if (result.isValid()) {

                // Access fiducial results
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    if (fr.getFiducialId() > 20 && fr.getFiducialId() < 24) {
                        apriltagId = fr.getFiducialId();
                    }

                }
                if (apriltagId == 22) { pattern = Pattern.PGP; }
                if (apriltagId == 23) { pattern = Pattern.PPG; }
                if (apriltagId == 21) { pattern = Pattern.GPP; }
            }
        }
        return pattern;
    }
    public List<Ball> getArtifactPosition(){

        List<Ball> result = new ArrayList<>();

        mLimelight.pipelineSwitch(mDetectionPipeline);
        LLResult llresult = mLimelight.getLatestResult();

        if (llresult != null) {

            //telemetry.addData(result.isValid());
            if (llresult.isValid()) {
                List<LLResultTypes.DetectorResult> detectorResults = llresult.getDetectorResults();
                for (LLResultTypes.DetectorResult dr : detectorResults) {

                    double pixelX = dr.getTargetXPixels();
                    double pixelY = dr.getTargetYPixels();
                    Point inputPixelPoint = new Point(pixelX, pixelY);
                    Point detectedMatPoint = mCalibration.distance(inputPixelPoint);
                    String detectedColor = dr.getClassName();
                    mTelemetry.addData("Detector", "Class: %s, Area: %.2f", dr.getClassName(), dr.getTargetArea());

                    Ball detected = new Ball(detectedColor, detectedMatPoint);
                    result.add(detected);
                }
            }
        }

        return result ;
    }

    public Pose3D getPosition(){

        Pose3D result = null;
        mLimelight.pipelineSwitch(mAprilTagPipeline);
        mTelemetry.addLine("running getPosition function");
        LLResult llresult = mLimelight.getLatestResult();
        if (llresult != null) {
            if (llresult.isValid()) {
                mTelemetry.addLine("result is valid");
                result = llresult.getBotpose();
            }
        }
        return result;

    }


}
