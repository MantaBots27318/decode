package org.firstinspires.ftc.teamcode.vision;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class Vision {

    public enum Pattern{
        GPP("GPP"),
        PPG("PPG"),
        PGP("PGP"),
        NONE("None");

        private final String mText;

        Pattern(String text) { mText = text; };

        public String text() { return mText; }
    }

    private Limelight3A  mLimelight;
    private Telemetry    mTelemetry;

    public Vision(Telemetry logger) {
        mTelemetry = logger;
        mTelemetry.addLine("Constructor");
    }

    public void initialize(HardwareMap hw) {
        mTelemetry.addLine("Initialize");
        mLimelight = hw.get(Limelight3A.class, "limelight");
        mLimelight.start();
    }

    public void close() {
        mLimelight.stop();
    }

    public Pattern readPattern() {

        Pattern pattern = Pattern.NONE;
        int apriltagId = 0;

        mLimelight.pipelineSwitch(0);
        LLResult result = mLimelight.getLatestResult();

        if (result != null) {



            //telemetry.addData(result.isValid());
            if (result.isValid()) {



                // Access fiducial results
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                mTelemetry.addLine("Found " + fiducialResults.size() + " April Tags");


                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    mTelemetry.addLine("" + fr.getFiducialId());
                    if (fr.getFiducialId() > 20 && fr.getFiducialId() < 24) {
                        apriltagId = fr.getFiducialId();
                    }

                }
                if (apriltagId == 22) {
                    pattern = Pattern.PGP;
                }
                if (apriltagId == 23) {
                    pattern = Pattern.PPG;
                }
                if (apriltagId == 21) {
                    pattern = Pattern.GPP;
                }
            }
        }
        return pattern;
    }
    public void getPosition(){

    }
    public void getArtifactPosition(){

    }


}
