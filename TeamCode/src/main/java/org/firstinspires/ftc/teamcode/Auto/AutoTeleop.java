package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import android.util.Pair;

import org.firstinspires.ftc.teamcode.FlyWheel.FlyWheelMotorComponent;
import org.firstinspires.ftc.teamcode.IntakeMotorComponent;
import org.firstinspires.ftc.teamcode.Turret.TurretPIDComponent;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


public abstract class AutoTeleop extends OpMode {

    private Follower follower;

    public enum State {
        LAUNCH,
        SHOOT,
        SHOOTTRAVEL,
        COLLECT,
        COLLECTTRAVEL
    }

    public State state = State.LAUNCH;

    private PathChain path;

    private final Pose startPose;
    private final Pose shootPose;

    private double targetX;
    private double targetY;


    private boolean shooting = false;

    private TurretPIDComponent turret;
    private FlyWheelMotorComponent launcher;

    private IntakeMotorComponent intake;

    private final Pair<Pose, Pose>[] collectPoses;
    private int ballSetIndex = -1;

    private final PathChain launchShootChain;
    private final PathChain[] shootCollectChain = new PathChain[3];
    private final PathChain[] collectShootChain = new PathChain[3];
    private final PathChain[] collectChain = new PathChain[3];

    AutoTeleop(Pose startPose, Pose shootPose, Pair<Pose, Pose>[] collectPoses, double targetX, double targetY) {
        this.startPose = startPose;
        this.shootPose = shootPose;
        this.collectPoses = collectPoses;
        this.targetX = targetX;
        this.targetY = targetY;

        launchShootChain = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        for (int i = 0; i<collectPoses.length; i++) {
            shootCollectChain[i] = follower.pathBuilder()
                    .addPath(new BezierCurve(shootPose, collectPoses[i].first))
                    .setLinearHeadingInterpolation(shootPose.getHeading(), collectPoses[i].first.getHeading())
                    .build();
            collectShootChain[i] = follower.pathBuilder()
                    .addPath(new BezierCurve(collectPoses[i].second, shootPose))
                    .setLinearHeadingInterpolation(collectPoses[i].second.getHeading(), shootPose.getHeading())
                    .build();
            collectChain[i] = follower.pathBuilder()
                    .addPath(new BezierLine(collectPoses[i].first, collectPoses[i].second))
                    .setLinearHeadingInterpolation(collectPoses[i].first.getHeading(), collectPoses[i].second.getHeading())
                    .build();
        }
    }


    @Override
    public void init() {
        if (startPose == null) {telemetry.addLine("startPose == null!!!"); stop();}
        state = State.LAUNCH;
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        turret = new TurretPIDComponent(hardwareMap, "turretMotor", 0.167, targetX, targetY, startPose, telemetry);
        launcher = new FlyWheelMotorComponent(hardwareMap, "flyWheelMotor");

        intake = new IntakeMotorComponent(hardwareMap, "intakeMotor");
    }

    @Override
    public void loop() {
        follower.update();
        if (!follower.isBusy() && !shooting) {
            transition();
        }

        if (shooting) {
            if (turret.aimToObject(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading()) <= 5) {

                // CODE FOR LAUNCH GOES HERE!!!

                transition();
            };
        }
    }

    public void transition() {
        // Transitions
        switch(state) {
            case LAUNCH:
                path = launchShootChain;
                state = State.SHOOTTRAVEL;
                break;

            case SHOOT:
                launcher.stopMotor();
                shooting = false;
                state = State.COLLECTTRAVEL;
                break;

            case SHOOTTRAVEL:
                state = State.SHOOT;
                break;

            case COLLECT:
                path = collectShootChain[ballSetIndex];
                intake.stopMotor();
                state = State.SHOOTTRAVEL;
                break;

            case COLLECTTRAVEL:
                state = State.COLLECT;
                break;

            default:
                state = State.LAUNCH;
                break;
        }

        // Actions
        switch (state) {
            case SHOOTTRAVEL:
                follower.followPath(path, true);
                launcher.runMotorAt(1);
                break;

            case SHOOT:
                shooting = true;
                break;

            case COLLECTTRAVEL:
                ballSetIndex++;
                if (ballSetIndex < collectPoses.length) {
                    path = collectShootChain[ballSetIndex];
                    follower.followPath(path, true);
                } else {
                    telemetry.addLine("Finished balls!");
                    stop();
                }
                break;

            case COLLECT:
                if (ballSetIndex < collectPoses.length) {
                    intake.startMotor();
                    path = collectChain[ballSetIndex];
                    follower.followPath(path, true);
                } else {
                    telemetry.addLine("Finished balls!");
                    stop();
                }
        }
    }

}
