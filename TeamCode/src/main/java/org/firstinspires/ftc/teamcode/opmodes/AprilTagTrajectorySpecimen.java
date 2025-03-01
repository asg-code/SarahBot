package org.firstinspires.ftc.teamcode.opmodes;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import android.util.Size;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "AprilTag Trajectory Specimen", group = "01-Test")
public class AprilTagTrajectorySpecimen extends LinearOpMode {

    /**
     * Variables to store the position and orientation of the camera on the robot. Setting these
     * values requires a definition of the axes of the camera and robot:
     *
     * Camera axes:
     * Origin location: Center of the lens
     * Axes orientation: +x right, +y down, +z forward (from camera's perspective)
     *
     * Robot axes (this is typical, but you can define this however you want):
     * Origin location: Center of the robot at field height
     * Axes orientation: +x right, +y forward, +z upward
     *
     * Position:
     * If all values are zero (no translation), that implies the camera is at the center of the
     * robot. Suppose your camera is positioned 5 inches to the left, 7 inches forward, and 12
     * inches above the ground - you would need to set the position to (-5, 7, 12).
     *
     * Orientation:
     * If all values are zero (no rotation), that implies the camera is pointing straight up. In
     * most cases, you'll need to set the pitch to -90 degrees (rotation about the x-axis), meaning
     * the camera is horizontal. Use a yaw of 0 if the camera is pointing forwards, +90 degrees if
     * it's pointing straight left, -90 degrees for straight right, etc. You can also set the roll
     * to +/-90 degrees if it's vertical, or 180 degrees if it's upside-down.
     */
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            2, -13, 9, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    // Locations of the tags on the field
    private Map<Integer, Pose2d> tagLocations = new HashMap<>();

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    // Declare subsystems here
    private MecanumDrive driveTrain;

    private boolean gp1ButtonALast = false;

    public AprilTagTrajectorySpecimen() {
        // Initialize tag locations
        initTagLocation();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initAprilTag();

        setManualExposure(6, 500);

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Get current field position based on AprilTag detection
        // Init drive train with the start position detected
        Pose2d startPose = new Pose2d(new Vector2d(12,-63), Math.toRadians(90));
        initSubsystems(startPose);

        // If Stop is pressed, exit OpMode
        if (isStopRequested()) return;

        telemetry.addLine("Press A for next step.");
        telemetry.update();

        while(!gp1GetButtonAPress() && opModeIsActive() && !isStopRequested()) {
            sleep(20);
        }
        telemetry.addLine("BUTTON CLICKED");
        telemetry.update();

        // Create a trajectory to first waypoint (somewhere we can see tag # 16)
        // Move forward 5 inches and turn 90 degrees CCW (heading 180 in RR coordinates)
        Action trajectoryAction = driveTrain.actionBuilder(startPose)
                .strafeTo(new Vector2d(startPose.position.x, startPose.position.y + 20))
                .build();

        Actions.runBlocking(new SequentialAction(trajectoryAction));
        

        Pose2d tempPose = new Pose2d(new Vector2d(startPose.position.x, startPose.position.y + 20), Math.toRadians(90));

        Action moveBack = driveTrain.actionBuilder(tempPose)
                .lineToY(startPose.position.y + 10)
                .turnTo(Math.toRadians(0))
                .build();

        Actions.runBlocking(new SequentialAction(moveBack));
        tempPose = new Pose2d(new Vector2d(12, -53), Math.toRadians(0));

        startPose = getFieldPosition(11);

        telemetry.addData("DriveTrain Initialized with Pose x y h (inch/deg)",
                "%.2f %.2f %.2f", startPose.position.x, startPose.position.y,
                Math.toDegrees(startPose.heading.toDouble()));
        telemetry.update();
        Action fillerName = driveTrain.actionBuilder(startPose)
                .turnTo(Math.toRadians(180))
                .strafeTo(new Vector2d(startPose.position.x - 23, startPose.position.y))
                .strafeTo(new Vector2d(startPose.position.x - 23, startPose.position.y - 25))
                .strafeTo(new Vector2d(startPose.position.x - 25, startPose.position.y - 25))
                .strafeTo(new Vector2d(startPose.position.x - 25, startPose.position.y - 5))
              //  .strafeToLinearHeading(new Vector2d(-48,-48), Math.toRadians(225))
             //   .strafeToLinearHeading(new Vector2d(-46,-36), Math.toRadians(180))
               // .lineToY(startPose.position.y + 5)
             //   .turnTo(Math.toRadians(180))
           //     .lineToX(-48)
              //  .lineToYConstantHeading(startPose.position.y + 5)
                .build();

        Actions.runBlocking(new SequentialAction(fillerName));
        tempPose =  new Pose2d(new Vector2d(42,-28), Math.toRadians(180));

        Action nextAction = driveTrain.actionBuilder(tempPose)
                        .strafeTo(new Vector2d(tempPose.position.x, tempPose.position.y -40))
                        .strafeTo(new Vector2d(tempPose.position.x, tempPose.position.y -30))
                        .build();

        startPose = getFieldPosition(16);
        telemetry.addData("DriveTrain Initialized with Pose x y h (inch/deg)",
                "%.2f %.2f %.2f", startPose.position.x, startPose.position.y,
                Math.toDegrees(startPose.heading.toDouble()));
        telemetry.update();


        Pose2d spikePose = new Pose2d(new Vector2d(-44, -53), Math.toRadians(90));
        Action toBucket1 = driveTrain.actionBuilder(new Pose2d(new Vector2d(-44, -53), Math.toRadians(90)))
                .lineToY(spikePose.position.y - 6)
                .turnTo(Math.toRadians(225))
                .build();

        Pose2d spikePose2 = new Pose2d(new Vector2d(-44, -59), Math.toRadians(225));
        Action secondSpike = driveTrain.actionBuilder(spikePose2)
                .turnTo(Math.toRadians(90))
                .strafeTo(new Vector2d(spikePose2.position.x - 10, spikePose2.position.y))
                .strafeTo(new Vector2d(spikePose2.position.x - 10, spikePose2.position.y + 6))
                .build();

        Pose2d moveToBucket = new Pose2d(new Vector2d(-54, -53), Math.toRadians(90));

        Action toBucket2 = driveTrain.actionBuilder(moveToBucket)
                .strafeTo(new Vector2d(moveToBucket.position.x - 10, moveToBucket.position.y - 4))
                .turnTo(Math.toRadians(225))
                .build();

        Action faceTag = driveTrain.actionBuilder(tempPose)
                .turnTo(Math.toRadians(180))
                .build();

        //assume u drop another here
        startPose = getFieldPosition(16);
        Action toPark = driveTrain.actionBuilder(startPose)
                .strafeTo(new Vector2d(startPose.position.x, startPose.position.y + 20))
                .strafeTo(new Vector2d(startPose.position.x + 20, startPose.position.y + 30))
                .build();


        // Run to waypoint


        // We should be able to see tag 16 now
//        Pose2d currentPose = getFieldPosition(16);
//        telemetry.addData("Second Pose x y h (inch/deg):",
//               "%.2f %.2f %.2f",
//                currentPose.position.x, currentPose.position.y,
//                Math.toDegrees(currentPose.heading.toDouble()));
//
//        telemetry.addLine("Press A for next step.");
//        telemetry.update();
//        while(!gp1GetButtonAPress() && opModeIsActive() && !isStopRequested()) {
//            sleep(20);
//        }

        // Create a trajectory to the next waypoint (tag 16)
        // Get to basket position with correct angle
//        trajectoryAction = driveTrain.actionBuilder(currentPose)
//                .splineTo(new Vector2d(-60, -60), Math.toRadians(-135))
//                .build();
        // Run to waypoint
     //   Actions.runBlocking(new SequentialAction(trajectoryAction));

        // Sit and loop till stop is pressed
        while (opModeIsActive() && !isStopRequested()) {

            // Update telemetry
            telemetry.update();
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();
    }

    private void initTagLocation() {
        tagLocations.put(11, new Pose2d(-72, 48, Math.toRadians(90)));
        tagLocations.put(12, new Pose2d(-0, 72, Math.toRadians(0)));
        tagLocations.put(13, new Pose2d(72, 48, Math.toRadians(270)));
        tagLocations.put(14, new Pose2d(72, -48, Math.toRadians(270)));
        tagLocations.put(15, new Pose2d(0, -72, Math.toRadians(180)));
        tagLocations.put(16, new Pose2d(-72, -40, Math.toRadians(90)));
    }

    private void initSubsystems(Pose2d startPose) {
        // Initialization code here

        driveTrain = new MecanumDrive(hardwareMap, startPose);
        telemetry.addData("DriveTrain Initialized with Pose x y h (inch/deg)",
                "%.2f %.2f %.2f", startPose.position.x, startPose.position.y,
                Math.toDegrees(startPose.heading.toDouble()));
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor.
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

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(1);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));


        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }

    /*
    Manually set the camera gain and exposure.
    This can only be called AFTER calling initAprilTag(), and only works for Webcams;
   */
    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

    private Pose2d getFieldPosition(int desiredTagID) {
        // Disable or re-enable the aprilTag processor at any time.
        visionPortal.setProcessorEnabled(aprilTag, true);

        List<AprilTagDetection> allDetections = new ArrayList<>();

        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        AprilTagDetection desiredTag = null; // The AprilTag target we are looking for

        // get 10 reading so we can average them out
        while (allDetections.size() < 10 && opModeIsActive()) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            targetFound = false;
            desiredTag  = null;

            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if (detection.id == desiredTagID) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is not in the library.
                    telemetry.addData("Skipping", "Tag ID %d is not in the library", detection.id);
                }
            }

            if (targetFound) {
                allDetections.add(desiredTag);
                if (allDetections.size() > 50) {
                    allDetections = allDetections.subList(0, 50);
                }
            }

            sleep(20); // Add a small delay to avoid overwhelming the CPU
        }
        visionPortal.setProcessorEnabled(aprilTag, false);

        // Calculate the average position of the tag
        if (allDetections.size() > 0) {
            double x = 0;
            double y = 0;
            double h = 0;

            for (AprilTagDetection detection : allDetections) {
                x += detection.robotPose.getPosition().x;
                y += detection.robotPose.getPosition().y;
                h += detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
            }

            x /= allDetections.size();
            y /= allDetections.size();
            h /= allDetections.size();

            return new Pose2d(x, y, Math.toRadians(h + 90));
        }

        return null;
    }

    private boolean gp1GetButtonAPress() {
        boolean isPressedButtonA = false;
        if (!gp1ButtonALast && gamepad1.a) {
            isPressedButtonA = true;
        }
        gp1ButtonALast = gamepad1.a;
        return isPressedButtonA;
    }

    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
            //don't even worry about it
        }
    }
}
