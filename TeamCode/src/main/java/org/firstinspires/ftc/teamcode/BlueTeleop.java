package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class BlueTeleop extends OpMode {
    private Follower follower;
    private boolean shooting = false;

    private boolean intaking = false;

    private GamepadEx gamepadEx1;
    private GamepadEx gamepadEx2;
    public static Pose startingPose = new Pose(12, 132, Math.toRadians(315));
    private boolean automatedDrive = false;
    private Supplier<PathChain> pathChain;

    private TurretPIDComponent turret;
    //private FlyWheelMotorComponent turret;

    private ServoKickComponent kicker1;
    //private ServoKickComponent kicker2;

    //private ServoKickComponent kicker3;

    private FlyWheelMotorComponent launcher;

    private IntakeMotorComponent intake;
    private TelemetryManager telemetryManager;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
        kicker1 = new ServoKickComponent(hardwareMap, "kicker1");
        //kicker2 = new ServoKickComponent(hardwareMap, "kicker2");
        //kicker3 = new ServoKickComponent(hardwareMap, "kicker3");

        //turret = new TurretPIDComponent(hardwareMap, "turretMotor", 0.167, -72, 72, startingPose, telemetry);

        turret = new TurretPIDComponent(hardwareMap, "turretMotor", 0.167, 0, 144, startingPose, telemetry);
        launcher = new FlyWheelMotorComponent(hardwareMap, "flyWheelMotor");

        intake = new IntakeMotorComponent(hardwareMap, "intakeMotor");

        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);


    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();
        gamepadEx1.readButtons();
        telemetryManager.update();
        turret.aimToObject();

        if (!automatedDrive) {
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepadEx1.getLeftY(),
                    gamepadEx1.getLeftX(),
                    -gamepadEx1.getRightX(),
                    true
            );

            else follower.setTeleOpDrive(
                    -gamepadEx1.getLeftY() * slowModeMultiplier,
                    gamepadEx1.getLeftX() * slowModeMultiplier,
                    -gamepadEx1.getRightX() * slowModeMultiplier,
                    true
            );

            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                slowMode = !slowMode;
            }

            if (gamepadEx1.wasJustPressed(PSButtons.CROSS) && !shooting) {
                launcher.runMotorAt(-1);
                shooting = true;
            }
            else if (gamepadEx1.wasJustPressed(PSButtons.CROSS)&& shooting){
                launcher.stopMotor();
                shooting = false;
            }


            if (gamepadEx1.wasJustPressed(PSButtons.CIRCLE)&&!intaking){
                intake.startMotor();
                intaking = true;
            }
            else if (gamepadEx1.wasJustPressed(PSButtons.CIRCLE)&&intaking){
                intake.stopMotor();
                intaking = false;
            }

            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
                kicker1.up();
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                kicker1.down();
            }
        }


        telemetryManager.debug("position", follower.getPose());
        telemetryManager.debug("velocity", follower.getVelocity());
        telemetryManager.debug("automatedDrive", automatedDrive);
    }
}
