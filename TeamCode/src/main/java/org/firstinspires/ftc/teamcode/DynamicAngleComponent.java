package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import java.lang.Math;


@Config
public class DynamicAngleComponent {

    public double coefficient;
    private Servo launchAngleServo;

    private double objectXPosition;

    private double objectYPosition;

    private double objectHeight;

    private double flyWheelRadius;

    private FlyWheelMotorComponent flyWheel;

    private AprilTagLocalization camera;
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);


    public DynamicAngleComponent(HardwareMap hardwareMap, String servoID, double objectXPosition, double objectYPosition, double objectHeight, double flyWheelRadius, Telemetry telemetry) {
        launchAngleServo = hardwareMap.servo.get(servoID);
        camera = new AprilTagLocalization(hardwareMap, cameraPosition, cameraOrientation, "Webcam 1", telemetry);
        flyWheel = new FlyWheelMotorComponent(hardwareMap, "flyWheelMotor");
        this.objectXPosition = objectXPosition;
        this.objectYPosition = objectYPosition;
        this.objectHeight = objectHeight;
        this.flyWheelRadius = flyWheelRadius;
    }


    public void turnServoBy(double degrees) {
        launchAngleServo.setPosition(degrees);
    }

    public void dynamicMotorPower() {
        double robotYPosition = camera.returnYPosition();
        double robotXPosition = camera.returnXPosition();
        double fixV = (2.0 * Math.PI * flyWheelRadius / 60.0) * 6000;


        double distance = Math.sqrt(Math.pow(objectXPosition - robotXPosition, 2) + Math.pow(objectYPosition - robotYPosition, 2));

        double denom = distance * Math.tan(Math.toRadians(45)) - objectHeight;
        double v = Math.sqrt((386.09 * Math.pow(distance, 2)) / (2.0 * Math.cos(Math.toRadians(45)) * Math.cos(Math.toRadians(45)) * denom));


        if (denom <= 0) {
            double inside = Math.pow(2.0 * fixV, 4) - 386.09 * (386.09 * Math.pow(distance, 2) + 2 * objectHeight * Math.pow(fixV, 2));
            double destinationAngleFlat = Math.atan((Math.pow(fixV, 2) - Math.sqrt(inside)) / (386.09 * distance));
            double destinationAngleArc = Math.atan((Math.pow(fixV, 2) + Math.sqrt(inside)) / (386.09 * distance));

            double chosen = destinationAngleFlat > 0 ? destinationAngleFlat : destinationAngleArc;
            turnServoBy(chosen % 360);
        } else {


            double rpm = (60.0 / (2.0 * Math.PI * flyWheelRadius)) * v * coefficient;

            double motorPower = rpm / 6000;

            flyWheel.runMotorAt(motorPower);
        }


    }

    public void stop(){
        flyWheel.stopMotor();
    }

}
