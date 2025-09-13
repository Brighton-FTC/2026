package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.acmerobotics.dashboard.FtcDashboard;

import java.util.List;

@Config
public class AprilTagLocalization {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

     /* Orientation:
     * If all values are zero (no rotation), that implies the camera is pointing straight up. In
     * most cases, you'll need to set the pitch to -90 degrees (rotation about the x-axis), meaning
     * the camera is horizontal. Use a yaw of 0 if the camera is pointing forwards, +90 degrees if
     * it's pointing straight left, -90 degrees for straight right, etc. You can also set the roll
     * to +/-90 degrees if it's vertical, or 180 degrees if it's upside-down.
     */

//    private Position cameraPosition = new Position(DistanceUnit.INCH,
//            0, 0, 0, 0);
//    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
//            0, -90, 0, 0);

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;


    public AprilTagLocalization(HardwareMap hardwareMap, Position cameraPosition, YawPitchRollAngles cameraOrientation, String webcamID){
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        aprilTag = new AprilTagProcessor.Builder()
                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)
                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, webcamID));
        builder.enableLiveView(true);
        builder.setCameraResolution(new Size(640, 480));
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);


        builder.addProcessor(aprilTag);


        visionPortal = builder.build();



    }

    public void startStreaming(){
        visionPortal.resumeStreaming();
    }

    public void stopStreaming(){
        visionPortal.stopStreaming();
    }

    public double returnYPosition() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        double posY = 0;
        for (AprilTagDetection detection : currentDetections) {
            posY = detection.robotPose.getPosition().y;
        }
        return posY;
    }
    public double returnXPosition() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        double posX = 0;
        for (AprilTagDetection detection : currentDetections) {
            posX = detection.robotPose.getPosition().x;
        }
        return posX;
    }

    public double returnYawPosition() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        double yaw = 0;
        for (AprilTagDetection detection : currentDetections) {
            yaw = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
        }
        return yaw;
    }

    public void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());



        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addData("Detection item", detection.id+ detection.metadata.name);
                // Only use tags that don't have Obelisk in them

                telemetry.addData("PositionX",
                            detection.robotPose.getPosition().x);
                telemetry.addData("PosY",
                            detection.robotPose.getPosition().y);
                telemetry.addData("PosZ",
                            detection.robotPose.getPosition().z);
                telemetry.addData("Pitch", detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES));
                telemetry.addData("Roll", detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES));
                telemetry.addData("Yaw", detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES));

            } else {
                telemetry.addData("ID", detection.id);
                telemetry.addData("Center", detection.center.x + detection.center.y);
            }
        }   // end for() loop

    }   // end method telemetryAprilTag()

    public void checkCase(){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                switch(detection.id){
                    case 20:
                        telemetry.addData("Obj", "Red");
                        break;
                    case 21:
                        telemetry.addData("Obj", "21");
                        break;
                    case 22:
                        telemetry.addData("Obj", "22");
                        break;
                    case 23:
                        telemetry.addData("Obj", "23");
                        break;
                    case 24:
                        telemetry.addData("Obj", "Blue");

                    default:
                        break;

                }
            }
        }

    }
}
