package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.FlyWheel.FlyWheelMotorComponent;
import org.firstinspires.ftc.teamcode.IntakeMotorComponent;
import org.firstinspires.ftc.teamcode.PSButtons;
import org.firstinspires.ftc.teamcode.ServoKick.ServoKickComponent;
import org.firstinspires.ftc.teamcode.Turret.TurretPIDComponent;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;


/*  FOLLOWING THE PRINCIPLES OF DRY (DON'T REPEAT YOURSELF),
    Code is modified to have an abstract generic teleop class.
    Any instance-specific variables MUST go into abstract functions
 */

@Configurable
@TeleOp
public abstract class GenericTeleop extends OpMode {
    public Follower follower;
    private boolean shooting = false;

    private boolean driveFieldCentric = false;

    private boolean intaking = false;

    private boolean aim = false;

    private GamepadEx gamepadEx1;
    private GamepadEx gamepadEx2;
    public final Pose startingPose = getStartingPose();
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

    protected abstract double getObjectXPosition();

    protected abstract Pose getStartingPose();

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
//
        turret = new TurretPIDComponent(hardwareMap, "turretMotor", 0.167, getObjectXPosition(), 144, startingPose, telemetry);
//        launcher = new FlyWheelMotorComponent(hardwareMap, "flyWheelMotor");
//
//        intake = new IntakeMotorComponent(hardwareMap, "intakeMotor");

        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        turret.resetTurretEncoder();
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

        if (!automatedDrive) {

            if(driveFieldCentric) {
                double forward = -gamepadEx1.getLeftY();
                double strafe = gamepadEx1.getLeftX();

                double heading = follower.getHeading();

                double rotX = forward * Math.cos(heading) + strafe * Math.sin(heading);
                double rotY = -forward * Math.sin(heading) + strafe * Math.cos(heading);
                if (!slowMode) follower.setTeleOpDrive(
                        rotY,
                        rotX,
                        gamepadEx1.getRightX(),
                        false
                );

                else follower.setTeleOpDrive(
                        rotY,
                        rotX,
                        gamepadEx1.getRightX() * slowModeMultiplier,
                        false
                );
            }
            else {
                if (!slowMode) follower.setTeleOpDrive(
                        gamepadEx1.getLeftY(),
                        -gamepadEx1.getLeftX(),
                        gamepadEx1.getRightX(),
                        true
                );

                else follower.setTeleOpDrive(
                        gamepadEx1.getLeftY() * slowModeMultiplier,

                        -gamepadEx1.getLeftX() * slowModeMultiplier,
                        gamepadEx1.getRightX() * slowModeMultiplier,
                        true
                );
            }

            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)&&!aim) {
                aim = true;
            }else if (gamepadEx1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)&&aim){
                aim = false;
            }
            if (aim){
                turret.aimToObject(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading());
            }

            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                slowMode = !slowMode;
            }
            if(gamepadEx1.wasJustPressed(PSButtons.SQUARE)){
                driveFieldCentric = !driveFieldCentric;
            }
//
//            if (gamepadEx1.wasJustPressed(PSButtons.CIRCLE) && !shooting) {
//                launcher.runMotorAt(1);
//                shooting = true;
//            }
//            else if (gamepadEx1.wasJustPressed(PSButtons.CIRCLE)&& shooting){
//                launcher.stopMotor();
//                shooting = false;
//            }
//
//
//            if (gamepadEx1.wasJustPressed(PSButtons.CROSS)&&!intaking){
//                intake.startMotor();
//                intaking = true;
//            }
//            else if (gamepadEx1.wasJustPressed(PSButtons.CROSS)&&intaking){
//                intake.stopMotor();
//                intaking = false;
//            }
//
        }


        telemetryManager.debug("position", follower.getPose());
        telemetryManager.debug("velocity", follower.getVelocity());
        telemetryManager.debug("automatedDrive", automatedDrive);
    }
}
