package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.outtakeLogic;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

//OUTDATED - USE BLUE1AUTO INSTEAD
@Autonomous
public class RedAutoRoutine1 extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;
    private outtakeLogic shooter = new outtakeLogic();
    private boolean shotsTriggered = false;

    public enum PathState {
        DRIVE_START_SHOOT,
        SHOOT_LOAD,
        DRIVE_SHOOT_END
    }

    PathState pathState;

    //poses
    private final Pose startPose = new Pose(21.751351351351364,124.95135135135135,Math.toRadians(142));
    private final Pose shootPose = new Pose(50.20540540540541,93.98918918918922,Math.toRadians(133));
    private final Pose endPose = new Pose(41.78918918918919,71.35135135135135,Math.toRadians(180));

    private PathChain driveStartShoot, driveShootEnd;

    public void buildPaths() {
        driveStartShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose,shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        driveShootEnd = follower.pathBuilder()
                .addPath(new BezierLine(shootPose,endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
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
                    if (!follower.isBusy()) {
                        if (!shotsTriggered) {
                            shooter.fire(3);
                            shotsTriggered = true;
                        }
                        else if (shotsTriggered && !shooter.isBusy()){
                            follower.followPath(driveShootEnd, true);
                            setPathState(PathState.DRIVE_SHOOT_END);
                            telemetry.addLine("Path 1 done");
                        }
                    }
                }
                break;
            case DRIVE_SHOOT_END:
                if(!follower.isBusy()) {
                    telemetry.addLine("DONE!");
                }
            default:
                telemetry.addLine("not in state");
                break;
        }
    }

    public void setPathState(PathState newState){
        pathState = newState;
        pathTimer.resetTimer();
        shotsTriggered = false;
    }

    @Override
    public void init() {
        pathState = PathState.DRIVE_START_SHOOT;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        //TODO: add other inits for other mechanisms
        shooter.init(hardwareMap);

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
        shooter.update();
        statePathUpdate();
        
        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x coord", follower.getPose().getX());
        telemetry.addData("y coord", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
    }
}
