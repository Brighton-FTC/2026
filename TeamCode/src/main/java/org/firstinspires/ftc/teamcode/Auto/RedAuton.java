package org.firstinspires.ftc.teamcode.Auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;


@Configurable
public class RedAuton extends GenericAutonomous {
    protected double getObjectXPosition() {
        return 0;
    }

    protected Pose getPickup1Pose() {
        return new Pose(37, 121, Math.toRadians(180));
    }

    protected Pose getPickup2Pose() {
        return new Pose(43, 130, Math.toRadians(180));
    }

    protected Pose getPickup3Pose() {
        return  new Pose(49, 135, Math.toRadians(180));
    }

    protected Pose getScorePose() {
        return new Pose(60, 85, Math.toRadians(135));
    }

    protected Pose getStartingPose() {
        return new Pose(22, 123, Math.toRadians(135));
    }

    protected Pose getControlPoint1(){return new Pose(60.661, 81.976);}

    protected Pose getControlPoint2(){return new Pose(56.366, 59.523);}

    protected Pose getControlPoint3(){return new Pose(55.740, 33.403);}
}
