package org.firstinspires.ftc.teamcode.opmodes;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import android.util.Size;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.HardwareConstants;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.GamepadController;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;


@TeleOp(name="AprilDistanceSpecimenPickup", group = "00-Teleop")
public class AprilDistanceSpecimenPickup extends LinearOpMode {
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
            0, 4, 9, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE_FROM_TAG = 12.0; //  this is how close the camera should get to the target (inches) (forward/back)
    final double MOVE_DISTANCE_LR = -14.0; //  How much left or right to move -ve is right, +ve is left from robot perspective
    final double MOVE_DISTANCE_FORWARD = 4.0; //  How much forward to move -ve is back, +ve is forward from robot perspective
    final double SAMPLE_PICKUP_DS_THRESHOLD = 3; //  How close to wall to get using distance sensor in inches
    final double DROP_OFF_DISTANCE_FROM_TAG = 24.0; //  this is how much backwards to move from the tag to drop off the specimen

    private static final int DESIRED_TAG_ID = 16;     // Choose the tag you want to approach or set to -1 for ANY tag.
      /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    private MecanumDrive driveTrain;
    private DistanceSensor robotDistanceSensor;
    private GamepadController gamepadController;

    @Override
    public void runOpMode() throws InterruptedException {

        initSubsystems();
        initAprilTag();
        setManualExposure(6, 500);

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        waitForStart();

        //telemetry.setAutoClear(true);
        AprilTagDetection desiredTag  = getAprilTagDetection(DESIRED_TAG_ID);

        // Tell the driver what we see, and what to do.
        if (desiredTag != null) {
            telemetry.addData("\n>","Press A to Drive to Target\n");
            telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
            telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
        }
        telemetry.update();

        // In auto - we don't have to wait for the driver to press the A button.
        // so the check for button press is not needed in auto.
        while(opModeIsActive() && !isStopRequested() && !gamepadController.gp1GetButtonAPress()) {

        }

        telemetry.setAutoClear(true);

        // Robot is somewhere in the field and facing Tag (DESIRE_TAG_ID)


        // Move the robot to the desired position using the April Tag
        moveJoyceForwardUsingAprilTag(DESIRED_TAG_ID, DESIRED_DISTANCE_FROM_TAG);

//        telemetry.addData(">", "Press A - Using roadrunner to move to the desired position");
//        telemetry.update();
//
//        while(opModeIsActive() && !isStopRequested() && !gamepadController.gp1GetButtonAPress()) {
//
//        }

        // Open Claw, lower arm in auto
        //claw.wristUp();
        //claw.intakeClawOpen();
        //slide.moveSlideLow();
        //arm.moveArmDown();

        // Move the robot closer to the wall
        TrajectoryActionBuilder closeToWall = driveTrain.actionBuilder(new Pose2d(0,0,Math.PI/2))
                .strafeTo(new Vector2d(-MOVE_DISTANCE_LR, 0))
                .setTangent(Math.PI/2)
                .lineToY(MOVE_DISTANCE_FORWARD);

        // Use roadrunner to move the robot to the desired position
        Actions.runBlocking(new SequentialAction(closeToWall.build()));

//        telemetry.addData(">", "Press A for next step - move forward to pickup position");
//        telemetry.update();
//        // Wait to X button to be pressed to end the opmode. (or timeout)
//        while(opModeIsActive() && !isStopRequested() && !gamepadController.gp1GetButtonAPress()) {
//
//        }

        // Move the robot to the wall pick up location using distance sensor
        moveJoyceForwardUsingDistanceSensor(SAMPLE_PICKUP_DS_THRESHOLD);

        // Close the claw
        // claw.intakeClawClose();
        // Simulate picking up
        safeWaitSeconds(3);

//        telemetry.addData(">", "Press A for next step - move back from wall");
//        telemetry.update();
//        // Wait to X button to be pressed to end the opmode. (or timeout)
//        while(opModeIsActive() && !isStopRequested() && !gamepadController.gp1GetButtonAPress()) {
//
//        }

        // Move back a bit using distance sensor
        // we are doubling the distance to move back so that we are far enough back to see an
        // april tag
        moveJoyceBackUsingDistanceSensor(SAMPLE_PICKUP_DS_THRESHOLD * 2);

//        telemetry.addData(">", "Press A for next step - move side to in front of april tag");
//        telemetry.update();
//        // Wait to X button to be pressed to end the opmode. (or timeout)
//        while(opModeIsActive() && !isStopRequested() && !gamepadController.gp1GetButtonAPress()) {
//
//        }

        // Build a trajectory to move away from the wall, towards the April Tag
        TrajectoryActionBuilder awayFromWall = driveTrain.actionBuilder(new Pose2d(0,0,Math.PI/2))
                .setTangent(0)
                .lineToX(MOVE_DISTANCE_LR);

        // Use roadrunner to move the robot in front of the tag
        Actions.runBlocking(new SequentialAction(awayFromWall.build()));

//        telemetry.addData(">", "Press A for next step - move back from april tag");
//        telemetry.update();
//        // Wait to X button to be pressed to end the opmode. (or timeout)
//        while(opModeIsActive() && !isStopRequested() && !gamepadController.gp1GetButtonAPress()) {
//
//        }

        // Move backwards to specimen drop off location using april tag
        moveJoyceBackdUsingAprilTag(DESIRED_TAG_ID, DROP_OFF_DISTANCE_FROM_TAG);

        // Turn to place the specimen
        // Use distance sensor movement to position close to chamber
        // place the specimen
        // Open the claw


        // Joyce is happy at this point

        telemetry.addLine("Joyce is Happy 😊 Press X to end OpMode");
        telemetry.update();
        // Wait to X button to be pressed to end the opmode. (or timeout)
        while(opModeIsActive() && !isStopRequested() && !gamepadController.gp1GetButtonXPress()) {

        }
        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();
    }

    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
            //don't even worry about it
        }
    }

    private void initSubsystems() throws InterruptedException {
        // Initialize all subsystems here

        telemetry.setAutoClear(false);

        // Init Pressed
        telemetry.addLine("Robot Init Pressed");
        telemetry.addLine("==================");
        telemetry.update();

        // Initialize drive train
        driveTrain = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        gamepadController = new GamepadController(gamepad1, gamepad2, driveTrain, this);
        // you can use this as a regular DistanceSensor.
        robotDistanceSensor = hardwareMap.get(DistanceSensor.class, HardwareConstants.RobotDistanceSensor);

        // Set the bulk mode to auto for control and expansion hubs
        // This optimizes the communication between the robot controller and the expansion hubs and
        // motors, sensors, etc. connected to them.
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        telemetry.addLine("Robot Init Completed ");
        telemetry.addLine("====================");
        telemetry.update();
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
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        visionPortal.setProcessorEnabled(aprilTag, false);

    }   // end method initAprilTag()

    /**
     * Add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                        detection.robotPose.getPosition().x,
                        detection.robotPose.getPosition().y,
                        detection.robotPose.getPosition().z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                        detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                        detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                        detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));

                 } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");

    }   // end method telemetryAprilTag()

    /*
   Manually set the camera gain and exposure.
   This can only be called AFTER calling initAprilTag(), and only works for Webcams;
  */
    private void setManualExposure(int exposureMS, int gain) {
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

    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
    private void moveRobot(double x, double y, double yaw) {
        // Implement your robot movement here
        driveTrain.setDrivePowers(new PoseVelocity2d(
                new Vector2d( x, y),
                yaw
        ));

        driveTrain.updatePoseEstimate();
    }

    private AprilTagDetection getAprilTagDetection(int tagId) {
        AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((tagId < 0) || (detection.id == tagId)) {
                    // Yes, we want to use this tag.
                    desiredTag = detection;
                    break;  // don't look any further.
                }
            }
        }

        return desiredTag;
    }

    private void moveJoyceForwardUsingAprilTag(int tagId, double offset) {
        // See https://ftc-docs.firstinspires.org/en/latest/apriltag/understanding_apriltag_detection_values/understanding-apriltag-detection-values.html#understanding-apriltag-detection-values
        // for how to interpret the values in the AprilTagDetection object.

        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)

        boolean done = false;               // Set to true when we are done with this task.

        visionPortal.setProcessorEnabled(aprilTag, true);
        while(opModeIsActive() && !isStopRequested() && !done)
        {
            AprilTagDetection desiredTag = getAprilTagDetection(tagId);
            if (desiredTag != null) {
                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double  rangeError      = (desiredTag.ftcPose.range - offset);
                double  headingError    = desiredTag.ftcPose.bearing;
                double  yawError        = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                telemetry.addData("Postion","Range Err %5.2f, Heading Err %5.2f, Yaw Err %5.2f ", rangeError, headingError, yawError);
                telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);

                // Apply desired axes motions to the drivetrain.
                moveRobot(drive, strafe, turn);

                // Check to see if we are close enough to the target.
                if (Math.abs(drive) <= 0.1 && Math.abs(strafe) <= 0.1 && Math.abs(turn) <= 0.1) {
                    done = true;
                }
            }

            telemetry.update();
            sleep(10);
        }

        moveRobot(0,0,0);
        visionPortal.setProcessorEnabled(aprilTag, false);
    }

    private void moveJoyceBackdUsingAprilTag(int tagId, double offset) {
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)
        double targetPosition   = 0;
        boolean done = false;               // Set to true when we are done with this task.

        visionPortal.setProcessorEnabled(aprilTag, true);
        AprilTagDetection desiredTag = getAprilTagDetection(tagId);
        if (desiredTag != null) {
            targetPosition = desiredTag.ftcPose.range + offset;
        }
        else {
            telemetry.addData("Error", "No April Tag detected");
            return;
        }

        telemetry.setAutoClear(true);
        while(opModeIsActive() && !isStopRequested() && !done)
        {
            desiredTag = getAprilTagDetection(tagId);
            if (desiredTag != null) {
                // Determine heading, range and Yaw (tag image rotation) error so we can use them to
                // control the robot automatically.

                // Since we are moving back, we need to reverse the range error
                double  rangeError      = (desiredTag.ftcPose.range - targetPosition);
                double  headingError    = desiredTag.ftcPose.bearing;
                double  yawError        = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
               // drive = -drive; // Move backwards

                turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                telemetry.addData("Offset Range", "%5.2f %5.2f inches", offset, desiredTag.ftcPose.range);
                telemetry.addData("Postion","Range Err %5.2f, Heading Err %5.2f, Yaw Err %5.2f ", rangeError, headingError, yawError);
                telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);

                // Apply desired axes motions to the drivetrain.
                moveRobot(drive, strafe, turn);

                // Check to see if we are close enough to the target.
                if (Math.abs(drive) <= 0.1 && Math.abs(strafe) <= 0.1 && Math.abs(turn) <= 0.1) {
                    //telemetry.addData("Error", "Done");
                    done = true;
                }
            }
            telemetry.update();
            sleep(10);
        }

        visionPortal.setProcessorEnabled(aprilTag, false);
        moveRobot(0,0,0);
    }

    private void moveJoyceForwardUsingDistanceSensor(double offset) {
        boolean done  = false;    // Set to true when we are done with this task.
        double drivePower = 0.3;        // Desired forward power/speed (-1 to +1)

        while(opModeIsActive() && !isStopRequested() && !done)
        {
            double distance = robotDistanceSensor.getDistance(DistanceUnit.INCH);
            telemetry.addData("Offset Distance", "%5.2f %5.2f inches", offset, distance);

            // Apply desired axes motions to the drivetrain.
            moveRobot(drivePower, 0, 0);

            // Check to see if we are close enough to the target.
            if ( distance <= offset) {
                done = true;
            }
            telemetry.update();
            sleep(10);
        }

        moveRobot(0, 0, 0);
    }

    private void moveJoyceBackUsingDistanceSensor(double offset) {
        boolean done = false;    // Set to true when we are done with this task.
        double drivePower = -0.3;        // Desired forward power/speed (-1 to +1)

        while(opModeIsActive() && !isStopRequested() && !done)
        {
            double distance = robotDistanceSensor.getDistance(DistanceUnit.INCH);
            telemetry.addData("Offset Distance", "%5.2f %5.2f inches", offset, distance);

            // Apply desired axes motions to the drivetrain.
            moveRobot(drivePower, 0, 0);

            // Check to see if we are close enough to the target.
            if ( distance >= offset) {
                done = true;
            }
            telemetry.update();
            sleep(10);
        }

        moveRobot(0, 0, 0);
    }
}
