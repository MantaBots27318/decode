package org.firstinspires.ftc.robotcore.internal.opmode;
public class OpModeMeta {
    public enum Flavor { TELEOP, AUTONOMOUS }
    public final String name;
    public final Flavor flavor;
    public final String group;
    private OpModeMeta(Builder b) { this.name = b.name; this.flavor = b.flavor; this.group = b.group; }
    public static class Builder {
        private String name = "";
        private Flavor flavor = Flavor.TELEOP;
        private String group = "";
        public Builder setName(String name) { this.name = name; return this; }
        public Builder setFlavor(Flavor flavor) { this.flavor = flavor; return this; }
        public Builder setGroup(String group) { this.group = group; return this; }
        public OpModeMeta build() { return new OpModeMeta(this); }
    }
}
