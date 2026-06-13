package com.seattlesolvers.solverslib.pedroCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
public class FollowPathCommand {
    public FollowPathCommand(Follower follower, PathChain path) {}
    public FollowPathCommand(Follower follower, PathChain path, boolean holdEnd) {}
    public FollowPathCommand(Follower follower, PathChain path, boolean holdEnd, double maxPower) {}
    public FollowPathCommand setGlobalMaxPower(double power) { return this; }
}
