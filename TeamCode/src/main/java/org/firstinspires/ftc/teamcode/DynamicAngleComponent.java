package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import java.lang.Math;

public class DynamicAngleComponent {

    private Servo launchAngleServo;

    private double scalingFactor;

    private double motorPower;

    private double objectXPosition;

    private double objectYPosition;

    private AprilTagLocalization camera;
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);



    public void DynamicAngleCompoent(HardwareMap hardwareMap, String servoID, double objectXPosition, double objectYPosition, double scalingFactor, double motorPower){
        launchAngleServo = hardwareMap.servo.get(servoID);
        camera = new AprilTagLocalization(hardwareMap, cameraPosition, cameraOrientation, "Webcam 1");
        this.objectXPosition = objectXPosition;
        this.objectYPosition = objectYPosition;
        this.scalingFactor = scalingFactor;
        this.motorPower = motorPower;
    }


    public double encoderTicksToAngle(int ticks) {
        return (ticks * scalingFactor);
    }

    public int angleToEncoderTicks(double degrees) {
        return (int) (degrees / scalingFactor);
    }

    public void turnServoBy(double degrees) {
        double currentPosition = launchAngleServo.getPosition();
        double TARGET_TICK_VALUE = angleToEncoderTicks(degrees) + currentPosition;

        launchAngleServo.setPosition((int) TARGET_TICK_VALUE);
    }




    public void aimToObject(){
        double robotYPosition = camera.returnYPosition();
        double robotXPosition = camera.returnXPosition();
        double destinationAngle = Math.asin((2*Math.sqrt(Math.pow(objectXPosition - robotXPosition, 2) + Math.pow(objectYPosition - robotYPosition, 2))*9.81)/Math.pow(motorPower, 2));

        turnServoBy(destinationAngle%360); // take the mod/remainder of toTurn/360
        // to keep the angle in the range of [0,360]
    }
}
