package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;


public class TurretComponent {

    private Follower follower;

    private Pose startingPose;

    private double scalingFactor;

    private double turretAngle;

    private double objectXPosition;

    private double objectYPosition;
    private Motor turretMotor;

    private AprilTagLocalization camera;
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    public TurretComponent(HardwareMap hardwareMap, String motorID, double scalingFactor, double objectXPosition, double objectYPosition, Pose startingPose, Telemetry telemetry) {
        turretMotor = new Motor(hardwareMap, motorID);
        turretMotor.resetEncoder();
        turretMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        camera = new AprilTagLocalization(hardwareMap, cameraPosition, cameraOrientation, "Webcam 1", telemetry);
        this.objectXPosition = objectXPosition;
        this.objectYPosition = objectYPosition;
        this.scalingFactor = scalingFactor;
        this.startingPose = startingPose;
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

    }

    public double encoderTicksToAngle(int ticks) {
        return (ticks * scalingFactor);
    }

    public int angleToEncoderTicks(double degrees) {
        return (int) (degrees / scalingFactor);
    }

    public void turnTurretBy(double degrees) {
        double currentPosition = turretMotor.getCurrentPosition();
        double TARGET_TICK_VALUE = angleToEncoderTicks(degrees) + currentPosition;

        turretMotor.setTargetPosition((int) TARGET_TICK_VALUE);
        turretMotor.setRunMode(Motor.RunMode.PositionControl);
        turretMotor.set(1);
    }

    public void aimToObject(){
        double robotYPosition = camera.returnYPosition();
        double robotXPosition = camera.returnXPosition();
        double robotAngle = follower.getHeading();
        double destinationAngle = Math.atan2(objectYPosition - robotYPosition,
                objectXPosition - robotXPosition);

        turretAngle = encoderTicksToAngle(turretMotor.getCurrentPosition());

        double toTurn = destinationAngle - (turretAngle+robotAngle);
        turnTurretBy(toTurn%360); // take the mod/remainder of toTurn/360
        // to keep the angle in the range of [0,360]
    }
}
