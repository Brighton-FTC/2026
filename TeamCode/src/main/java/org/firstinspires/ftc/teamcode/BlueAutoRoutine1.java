package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class BlueAutoRoutine1 extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        DRIVE_START_SHOOT,
        SHOOT_LOAD
    }

    PathState pathState;

    //poses
    private final Pose startPose = new Pose(21.751351351351364,124.95135135135135,Math.toRadians(142));
    private final Pose shootPose = new Pose(50.20540540540541,93.98918918918922,Math.toRadians(133));

    private PathChain driveStartShoot;

    public void buildPaths() {
        driveStartShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose,shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_START_SHOOT:
                follower.followPath(driveStartShoot,true);
                setPathState(PathState.SHOOT_LOAD);
                break;
            case SHOOT_LOAD:
                if (!follower.isBusy()) {
                    //TODO: add shooter logic here
                    telemetry.addLine("Path 1 done");
                }
                break;
            default:
                telemetry.addLine("not in state");
                break;
        }
    }

    public void setPathState(PathState newState){
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathState = PathState.DRIVE_START_SHOOT;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        //TODO: add other inits for other mechanisms
        buildPaths();
        follower.setPose(startPose);
    }

    public void start(){
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();
        
        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x coord", follower.getPose().getX());
        telemetry.addData("y coord", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
    }
}
