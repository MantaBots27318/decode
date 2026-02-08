package org.firstinspires.ftc.teamcode.vision;

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

    public String text()       { return mText;       }
    public int    identifier() { return mIdentifier; }
}

