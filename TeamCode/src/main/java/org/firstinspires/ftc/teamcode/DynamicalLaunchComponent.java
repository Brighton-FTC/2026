package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import java.lang.Math;

public class DynamicalLaunchComponent {

    private Servo launchAngleServo;

    private double scalingFactor;

    private double objectHeight;

    private double objectXPosition;
    
    private double launchVelocity;

    private double objectYPosition;

    private double launchAngle;

    private AprilTagLocalization camera;
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);



    public void DynamicAngleCompoent(HardwareMap hardwareMap, String servoID, double objectXPosition, double objectYPosition, double scalingFactor, double objectHeight, double launchVelocity){
        launchAngleServo = hardwareMap.servo.get(servoID);
        camera = new AprilTagLocalization(hardwareMap, cameraPosition, cameraOrientation, "Webcam 1");
        this.objectXPosition = objectXPosition;
        this.objectYPosition = objectYPosition;
        this.scalingFactor = scalingFactor;
        this.objectHeight = objectHeight;
        this.launchVelocity = launchVelocity;
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




    public void dynamicLuanchAngle(){
        double robotYPosition = camera.returnYPosition();
        double robotXPosition = camera.returnXPosition();
        double distance = Math.sqrt(Math.pow(robotXPosition, 2) + Math.pow(robotYPosition, 2));
        double inside = Math.pow(launchVelocity, 4) - 9.81 * (9.81 * Math.pow(distance, 2) + 2 * objectHeight*Math.pow(launchVelocity, 2));

        double sqrtTerm = Math.sqrt(inside);

        // Two possible angles:
        double tanTheta1 = (Math.pow(launchVelocity, 2) + sqrtTerm) / (9.81 * distance);
        double tanTheta2 = (Math.pow(launchVelocity, 2) - sqrtTerm) / (9.81 * distance);

        double theta1 = Math.atan(tanTheta1);
        double theta2 = Math.atan(tanTheta2);
    }

    public void dynamicMotorPower(){
        double robotYPosition = camera.returnYPosition();
        double robotXPosition = camera.returnXPosition();
        double distance = Math.sqrt(Math.pow(robotXPosition, 2) + Math.pow(robotYPosition, 2));
        double denom = distance * Math.tan(launchAngle) - objectHeight;
        double launchVelocity = Math.sqrt((9.81 * Math.pow(distance, 2)) / (2.0 * Math.cos(launchAngle) * Math.cos(launchAngle) * denom));

        telemetry.addData("launch launchVelocity", launchVelocity);
    }
}
