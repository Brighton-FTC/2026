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
@Autonomous
public class BlueAuton extends GenericAutonomous {
    protected double getObjectXPosition() {
        return 0;
    }

    protected Pose getPickup1Pose() {
        return new Pose(22, 84, Math.toRadians(180));
    }

    protected Pose getPickup2Pose() {
        return new Pose(22, 60, Math.toRadians(180));
    }

    protected Pose getPickup3Pose() {
        return  new Pose(22, 35.5, Math.toRadians(180));
    }

    protected Pose getScorePose() {
        return new Pose(60, 84, Math.toRadians(180));
    }
    protected Pose getStartScorePose(){
        return new Pose(44, 100, Math.toRadians(135));
    }

    protected Pose getStartingPose() {
        return new Pose(22, 123, Math.toRadians(135));
    }

    protected Pose getControlPoint1(){return new Pose(60.661, 81.976);}

    protected Pose getControlPoint2(){return new Pose(56.366, 59.523);}

    protected Pose getControlPoint3(){return new Pose(55.740, 33.403);}
}