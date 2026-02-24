package org.firstinspires.ftc.teamcode;


public enum AutonomousStep {

    GPP("GPP", 0),
    PPG("PPG", 1),
    PGP("PGP", 2),
    NONE("None", -1);

    private final String mText;
    private final int    mIdentifier;
    AutonomousStep mAutonomousStep;

    AutonomousStep(String text, int identifier) {
        mText = text;
        mIdentifier = identifier;
    };

    public String text()       { return mText;       }
    public int    identifier() { return mIdentifier; }

    public AutonomousStep next() {
        if(mIdentifier == AutonomousStep.NONE.mIdentifier ) { return AutonomousStep.GPP; }
        if(mIdentifier == AutonomousStep.GPP.mIdentifier ) { return AutonomousStep.PPG; }
        if(mIdentifier == AutonomousStep.PPG.mIdentifier ) { return AutonomousStep.PGP; }
        if(mIdentifier == AutonomousStep.PGP.mIdentifier ) { return AutonomousStep.NONE; }
        return AutonomousStep.NONE;
    }

    public AutonomousStep previous() {
        if(mIdentifier == AutonomousStep.NONE.mIdentifier ) { return AutonomousStep.PGP; }
        if(mIdentifier == AutonomousStep.GPP.mIdentifier ) { return AutonomousStep.NONE; }
        if(mIdentifier == AutonomousStep.PPG.mIdentifier ) { return AutonomousStep.GPP; }
        if(mIdentifier == AutonomousStep.PGP.mIdentifier ) { return AutonomousStep.PPG; }
        return AutonomousStep.NONE;
    }
}


