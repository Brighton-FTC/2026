package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10)
            .forwardZeroPowerAcceleration(-36.217454168514266)
            .lateralZeroPowerAcceleration(-54.78063416685368)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.037, 0, 0.002, 0.02))
            .headingPIDFCoefficients(new PIDFCoefficients(0.8, 0, 0.01, 0.025))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.008, 0, 0.001,0.6, 0.01))
            .centripetalScaling(0.05);


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1.5, 1);


    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName("lf")
            .rightFrontMotorName("rf")
            .leftRearMotorName("lr")
            .rightRearMotorName("rr")

            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)

            .xVelocity(63.76887812576896)
            .yVelocity(54.06007829801306)

            .maxPower(1);


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
//                .twoWheelLocalizer(localizerConstants)
                .build();

    }


    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(8)
            .strafePodX(-5)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

//    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
//            .forwardEncoder_HardwareMapName("lf")
//            .strafeEncoder_HardwareMapName("rr")
//            .IMU_HardwareMapName("imu")
//            .IMU_Orientation(
//                    new RevHubOrientationOnRobot(
//                            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
//                            RevHubOrientationOnRobot.UsbFacingDirection.UP
//                    )
//            );

}
