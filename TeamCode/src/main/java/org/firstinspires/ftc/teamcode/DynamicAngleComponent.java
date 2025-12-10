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

    public double coefficient = 1;
    private Servo launchAngleServo;

    private static final double MIN_POSITION = 0;

    private static final double MAX_POSITION = 180;

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
        launchAngleServo.setDirection(Servo.Direction.REVERSE);
        camera = new AprilTagLocalization(hardwareMap, cameraPosition, cameraOrientation, "Webcam 1", telemetry);
        flyWheel = new FlyWheelMotorComponent(hardwareMap, "flyWheelMotor");
        this.objectXPosition = objectXPosition;
        this.objectYPosition = objectYPosition;
        this.objectHeight = objectHeight;
        this.flyWheelRadius = flyWheelRadius;
    }

    //Gear ratio for servo gear vs launcher gear
    //setPos (Gr*degrees/ppd)/180
    public void turnServoTo(double degrees) {
        //Gear ratio = 2, launch angle is set to 60 degrees when servo is at neutral position.
        //Servo constrained [0, 90], launch angle constrained [45, 90]
        launchAngleServo.setPosition((2*(degrees-60))/300);
    }

    public void resetServo(){
        launchAngleServo.setPosition(0);
    }

    public void dynamicMotorPower() {

        /*
        Assuming turret faces the goal at all time, therefore 3D kinematics may be neglected.
        2 Modes of launching artifacts - adjusting speed or adjusting angle (adjusting speed comes in priority)

        Launch mode is switched to dynamic angle if / when it is unreachable at 60 degrees


         */
        double robotYPosition = camera.returnYPosition();
        double robotXPosition = camera.returnXPosition();

        if (robotXPosition != 0 && robotYPosition != 0){


            double fixV = (2.0 * Math.PI * flyWheelRadius / 60.0) * 6000;


            double distance = Math.sqrt(Math.pow(objectXPosition - robotXPosition, 2) + Math.pow(objectYPosition - robotYPosition, 2));
            
            //We let y = objectHeight and x = distance from robot
            double denom = distance * Math.tan(Math.toRadians(60)) - objectHeight;
            
            //Linear velocity required for artifact to pass through x = distance from robot and y = object height
            double v = Math.sqrt((386.09 * Math.pow(distance, 2)) / (2.0 * Math.cos(Math.toRadians(60)) * Math.cos(Math.toRadians(60)) * denom));

            //Denominator less than or equal 0 will yield undefined / imaginary solution. Meaning no velocity will allow artifact to reach target at 60 degrees.
            if (denom <= 0) {
                double inside = Math.pow(fixV, 4) - 386.09 * (386.09 * Math.pow(distance, 2) + 2 * objectHeight * Math.pow(fixV, 2));

                //Solving the quadratic yields 2 roots of trajectory in different shapes.
                double destinationAngleFlat = Math.atan((Math.pow(fixV, 2) - Math.sqrt(inside)) / (386.09 * distance));
                double destinationAngleArc = Math.atan((Math.pow(fixV, 2) + Math.sqrt(inside)) / (386.09 * distance));

                double chosen = Math.max(destinationAngleFlat, destinationAngleArc);
                turnServoTo(Math.toDegrees(chosen) % 360);
            } else {

                //This ensures launch angle is reset to 60 when the launcher is running in dynamic velocity mode.
                resetServo();

                //Linear velocity is converted to angular velocity.
                double rpm = (60.0 / (2.0 * Math.PI * flyWheelRadius)) * v * coefficient;

                double motorPower = rpm / 6000;

                flyWheel.runMotorAt(motorPower);
            }
        }

    }

    public void stop(){
        flyWheel.stopMotor();
    }

}
