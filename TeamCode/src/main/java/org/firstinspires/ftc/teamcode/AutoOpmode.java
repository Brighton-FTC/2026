package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import android.util.Pair;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
/*
I made this by looking at both the pedropathing docs in their example code section,
as well as some ideas from subzero and their auto, which uses a similar opmode
specifically for their robot that can be used in multiple cases to reduce repeated
code. Still need to add other hardware like intake and shooting logic.
If this doesn't work then it is still a good learning exercise
 */
@Autonomous(name="Practice Auto")
public class AutoOpmode extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        DRIVE_START_SHOOT,
        SHOOT_COLLECT_1, // this means go from shoot to collect startpoint 1
        COLLECT_1, // this means do the collect path
        COLLECT_1_SHOOT, // this means go from end of collect point to shoot pos
        SHOOT_COLLECT_2,
        COLLECT_2,
        COLLECT_2_SHOOT,
        SHOOT_COLLECT_3,
        COLLECT_3,
        COLLECT_3_SHOOT,
        IDLE
    }

    protected PathState pathState;

    private final Pose startPose;
    private final Pose shootPose;
    private final Pair<Pose, Pose>[] collectPoses;

    private PathChain driveStartShoot;
    private PathChain[] collectPaths = new PathChain[3];
    private PathChain[] collectToShootPaths = new PathChain[3];
    private PathChain[] shootToCollectPaths = new PathChain[3];

    public AutoOpmode() {
        // default values - will be overridden by subclasses
        this(new Pose(0, 0, 0), new Pose(0, 0, 0), new Pair[0]);
    }
    // this is needed because the public constructor cannot take in any parameters
    protected AutoOpmode(Pose startPose, Pose shootPose, Pair<Pose, Pose>[] collectPoses){
        this.startPose = startPose;
        this.shootPose = shootPose;
        this.collectPoses = collectPoses;
    }


    public void buildPaths(){
        driveStartShoot = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        for (int i = 0; i < 3; i++){
            collectPaths[i] = follower.pathBuilder()
                    .addPath(new BezierCurve(collectPoses[i].first, collectPoses[i].second))
                    .setLinearHeadingInterpolation(collectPoses[i].first.getHeading(), collectPoses[i].second.getHeading())
                    .build();

            collectToShootPaths[i] = follower.pathBuilder()
                    .addPath(new BezierCurve(collectPoses[i].second, shootPose))
                    .setLinearHeadingInterpolation(collectPoses[i].second.getHeading(), shootPose.getHeading())
                    .build();

            shootToCollectPaths[i] = follower.pathBuilder()
                    .addPath(new BezierCurve(shootPose, collectPoses[i].first))
                    .setLinearHeadingInterpolation(shootPose.getHeading(), collectPoses[i].first.getHeading())
                    .build();
        }
    }

    public void statePathUpdate(){
        switch (pathState){
            case DRIVE_START_SHOOT:
                follower.followPath(driveStartShoot, true);
                setPathState(PathState.SHOOT_COLLECT_1);
                break;
            case SHOOT_COLLECT_1:
                if(!follower.isBusy()){
                    follower.followPath(shootToCollectPaths[0], true);
                    setPathState(PathState.COLLECT_1);
                }
                break;
            case COLLECT_1:
                if(!follower.isBusy()){
                    //intake on
                    follower.followPath(collectPaths[0], true);
                    setPathState(PathState.COLLECT_1_SHOOT);
                }
                break;
            case COLLECT_1_SHOOT:
                if(!follower.isBusy()){
                    follower.followPath(collectToShootPaths[0], true);
                    //shoot
                    setPathState(PathState.SHOOT_COLLECT_2);

                }
                break;
            case SHOOT_COLLECT_2:
                if(!follower.isBusy()){
                    follower.followPath(shootToCollectPaths[1], true);
                    setPathState(PathState.COLLECT_2);
                }
                break;
            case COLLECT_2:
                //intake on
                if(!follower.isBusy()){
                    follower.followPath(collectPaths[1], true);
                    setPathState(PathState.COLLECT_2_SHOOT);
                }
                break;
            case COLLECT_2_SHOOT:
                if(!follower.isBusy()){
                    follower.followPath(collectToShootPaths[1], true);
                    //shoot
                    setPathState(PathState.SHOOT_COLLECT_3);
                }
                break;
            case SHOOT_COLLECT_3:
                if(!follower.isBusy()){
                    follower.followPath(shootToCollectPaths[2], true);
                    setPathState(PathState.COLLECT_3);
                }
                break;
            case COLLECT_3:
                //intake on
                if(!follower.isBusy()){
                    follower.followPath(collectPaths[2], true);
                    setPathState(PathState.COLLECT_3_SHOOT);
                }
                break;
            case COLLECT_3_SHOOT:
                if(!follower.isBusy()){
                    follower.followPath(collectToShootPaths[2], true);
                    //shoot
                    setPathState(PathState.IDLE); // or go to wherever
                }
                break;
            case IDLE:
                break;
        }
    }

    @Override
    public void init() {
        if(startPose == null){
            telemetry.addLine("No startPose added");
            return;
        }

        pathState = PathState.DRIVE_START_SHOOT;
        pathTimer = new Timer();
        opModeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();
    }

    @Override
    public void start(){
        opModeTimer.resetTimer();
        setPathState(PathState.DRIVE_START_SHOOT);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    public void setPathState(PathState newState){
        pathState = newState;
        pathTimer.resetTimer();
    }
}