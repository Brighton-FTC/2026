package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.AprilTagLocalization;


@Config
@Configurable
public class TurretPIDComponent {

    private Follower follower;
    public static double kP = 0.0017;
    public static double kI = 0.0;
    public static double kD = 0.0;

    private double scalingFactor;

    private double turretAngle;

    private double objectXPosition;

    private double objectYPosition;
    private Motor turretMotor;
    private final MultipleTelemetry telemetry;

    private AprilTagLocalization camera;
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    private final PIDController controller = new PIDController(0, 0, 0);

    public TurretPIDComponent(HardwareMap hardwareMap, String motorID, double scalingFactor, double objectXPosition, double objectYPosition, Telemetry telemetry) {
        turretMotor = new Motor(hardwareMap, motorID);
        turretMotor.resetEncoder();
        controller.setPID(kP, kI, kD);
        //remove if
        turretMotor.setDistancePerPulse(4*scalingFactor); // 360/537.7 = 4*0.167
        turretMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

//        camera = new AprilTagLocalization(hardwareMap, cameraPosition, cameraOrientation, "Webcam 1", telemetry);
        this.objectXPosition = objectXPosition;
        this.objectYPosition = objectYPosition;
        this.scalingFactor = scalingFactor;
        FtcDashboard dashboard = FtcDashboard.getInstance();
        this.telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
//        follower.update();

    }

    public void resetTurretEncoder(){
        turretMotor.resetEncoder();
    }

    public double encoderTicksToAngle(int ticks) {
        return (ticks * scalingFactor);
        //return (((double) ticks /4)*scalingFactor);
    }

    //Gear Ratio is 0.25
    //Turret degree per motor revolution is 360*0.25 = 90
    //PPR for GoBuilda Yellow Jacket = 537.7
    public int angleToEncoderTicks(double degrees) {
        return (int) (degrees / scalingFactor);
        //return (int) ((degrees*4)/scalingFactor);
    }

    public void turnTurretBy(double degrees) {


        double currentPosition = turretMotor.getCurrentPosition();
        double TARGET_TICK_VALUE = angleToEncoderTicks(degrees); // + currentPosition;
        controller.setSetPoint(TARGET_TICK_VALUE);
        double power = controller.calculate(currentPosition);

        telemetry.addData("Motor Power: ", power);
        telemetry.update();

        turretMotor.set(-power);

        if (controller.getPositionError() < 5){
            turretMotor.set(0);}
    }



    public void aimToObject(double robotX, double robotY, double robotHeading) {
        //double robotYPosition = camera.returnYPosition();
        //double robotXPosition = camera.returnXPosition();
        //follower.getPose().getY();
        //follower.getPose().getX();
        if (robotX != 1000 && robotY != 1000) {
            double robotAngle = Math.toDegrees(robotHeading);
            double destinationAngle = Math.toDegrees(Math.atan2(objectYPosition - robotY,
                    objectXPosition - robotX));

            turretAngle = encoderTicksToAngle(turretMotor.getCurrentPosition());

            double toTurn = destinationAngle - (turretAngle + robotAngle);
            telemetry.addData("To turn :", toTurn);
            telemetry.update();
            double turnMod = (((toTurn + 540) % 360) - 180);


            // take the mod/remainder of toTurn/360 to keep angle in the range of [0,360]
            if (turnMod + encoderTicksToAngle(turretMotor.getCurrentPosition()) > 180) {
                turnTurretBy(turnMod-360);
            } else if (turnMod + encoderTicksToAngle(turretMotor.getCurrentPosition()) < -180) {
                turnTurretBy(turnMod+360);
            } else {
                turnTurretBy(turnMod);
            }
//           turnTurretBy(turnMod); // isn't this turning twice?
        }
    }

}
