package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.FlyWheel.DynamicAngleComponent;
import org.firstinspires.ftc.teamcode.FlyWheel.FlyWheelMotorComponent;
import org.firstinspires.ftc.teamcode.IntakeMotorComponent;
import org.firstinspires.ftc.teamcode.ServoKick.ServoKickComponent;
import org.firstinspires.ftc.teamcode.Turret.TurretPIDComponent;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;


public abstract class GenericAutonomous extends OpMode {

    public Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private boolean intaking = false;
    public final Pose startingPose = getStartingPose();

    private final Pose startScorePose = getStartScorePose();
    private final Pose scorePose = getScorePose();
    private final Pose pickup1Pose = getPickup1Pose();
    private final Pose pickup2Pose = getPickup2Pose();
    private final Pose pickup3Pose = getPickup3Pose();

    private final Pose controlPoint1 = getControlPoint1();
    private final Pose controlPoint2 = getControlPoint2();
    private final Pose controlPoint3 = getControlPoint3();


    private TurretPIDComponent turret;
    private DynamicAngleComponent launcher;
    private FlyWheelMotorComponent transfer;
    private IntakeMotorComponent intake;

    private ServoKickComponent cap;

    protected abstract double getObjectXPosition();
    protected abstract Pose getStartingPose();
    protected abstract Pose getScorePose();
    protected abstract Pose getPickup1Pose();
    protected abstract Pose getPickup2Pose();
    protected abstract Pose getPickup3Pose();
    protected abstract Pose getControlPoint1();
    protected abstract Pose getControlPoint2();
    protected abstract Pose getControlPoint3();
    protected abstract Pose getStartScorePose();

    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startingPose, startScorePose));
        scorePreload.setLinearHeadingInterpolation(startingPose.getHeading(), pickup1Pose.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(startScorePose, controlPoint1, pickup1Pose))
                .setConstantHeadingInterpolation(pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setConstantHeadingInterpolation(pickup1Pose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, controlPoint2, pickup2Pose))
                .setConstantHeadingInterpolation(pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setConstantHeadingInterpolation(pickup1Pose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, controlPoint3, pickup3Pose))
                .setConstantHeadingInterpolation(pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setConstantHeadingInterpolation(pickup1Pose.getHeading())
                .build();
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

        turret = new TurretPIDComponent(hardwareMap, "turretMotor", 0.167, getObjectXPosition(), 144, telemetry);
//        launcher = new FlyWheelMotorComponent(hardwareMap, "flyWheelMotor");

        intake = new IntakeMotorComponent(hardwareMap, "intakeMotor");

        cap = new ServoKickComponent(hardwareMap, "launchCap");

        transfer = new FlyWheelMotorComponent(hardwareMap, "transferMotor");

        launcher = new DynamicAngleComponent(hardwareMap, "servo", getObjectXPosition(), 144, 42,2, 0.2, startingPose, telemetry);

    }


    public void pathUpdate(){
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                cap.down();
                setState(1);
                break;
            case 1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    intaking = !intaking;
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup1,true);
                    intaking = !intaking;
                    setState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup1,true);
                    intaking = !intaking;
                    transfer.runMotorAt(1);
                    cap.up();
                    setState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    cap.down();
                    /* Score Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup2,true);
                    intaking = !intaking;
                    setState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup2,true);
                    intaking = !intaking;
                    transfer.runMotorAt(1);
                    cap.up();
                    setState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    cap.down();
                    /* Score Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup3,true);
                    intaking = !intaking;
                    setState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    intaking = !intaking;
                    transfer.runMotorAt(1);
                    cap.up();
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup3, true);
                    setState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setState(-1);
                }
                break;
        }
    }

    public void setState(int state){
        pathState = state;
    }
    @Override
    public void loop(){

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        turret.aimToObject(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading());
        launcher.dynamicMotorPower();

        if (intaking){
            intake.startMotor();
        }
        else{intake.stopMotor();}

        pathUpdate();
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setState(0);
    }


}
