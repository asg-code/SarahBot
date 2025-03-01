package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.GamepadController;

@TeleOp(name = "Normal TeleOp", group = "00-Teleop")
public class NormalTeleOp extends LinearOpMode {

    private GamepadController gamepadController;
    // Declare subsystems here
    private MecanumDrive driveTrain;

    private enum ALLIANCE {
        BLUE,
        RED
    }

    private ALLIANCE allianceSelection = ALLIANCE.RED;

    // We can tranfer this from last autonoumous opmode if needed,
    // but most the time we don't need to.
    private Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialization code here
        initSubsystems();

       /* if(gamepadController.gp1GetX()) {
            claw.allianceColor = "BLUE";
        } else if (gamepadController.gp1GetX()) {
            claw.allianceColor = "RED";
        }*/

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        telemetry.addLine("X for BLUE ALLIANCE : O for RED ALLIANCE");
        if (gamepadController.gp1GetX()) {
            allianceSelection = ALLIANCE.BLUE;
        } else if (gamepadController.gp1GetX()) {
            allianceSelection = ALLIANCE.RED;
        }

        telemetry.addLine("Start Pressed");
        telemetry.update();

        // If Stop is pressed, exit OpMode
        if (isStopRequested()) return;

        /*If Start is pressed, enter loop and exit only when Stop is pressed */
        while (!isStopRequested()) {
            outputTelemetry();
            telemetry.update();

            while (opModeIsActive()) {
                // TeleOp code here
                gamepadController.runSubSystems();
                outputTelemetry();
            }
        }
    }

    private void initSubsystems() throws InterruptedException {
        // Initialize all subsystems here

        telemetry.setAutoClear(false);

        // Init Pressed
        telemetry.addLine("Robot Init Pressed");
        telemetry.addLine("==================");
        telemetry.update();

        // Intialize drive train
        driveTrain = new MecanumDrive(hardwareMap, startPose);


        gamepadController = new GamepadController(gamepad1, gamepad2,
                driveTrain, this);

        gamepadController.driveType = GamepadController.DriveType.ROBOT_CENTRIC;
        telemetry.addLine("Gamepad Initialized");
        telemetry.update();


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
     * Output telemetry messages to the driver station
     */
    public void outputTelemetry() {

        telemetry.setAutoClear(true);
        telemetry.addLine("Running Normal TeleOpMode");

        // Output telemetry messages for susbsystems here
        telemetry.addData("Drive Type : ", gamepadController.driveType);
//        telemetry.addLine("Localizer pose data:");
//        Pose2d pose = driveTrain.getEstimatedPose();
//        telemetry.addData("X", pose.position.x);
//        telemetry.addData("Y", pose.position.y);
//        telemetry.addData("Heading (deg)", Math.toDegrees(pose.heading.toDouble()));

        telemetry.addData("Parallel Left /LF Encoder", driveTrain.leftFront.getCurrentPosition());
        telemetry.addData("Parallel Right / RF Encoder", driveTrain.rightFront.getCurrentPosition());
        telemetry.addData("Perpendicular / RB Encoder", driveTrain.rightBack.getCurrentPosition());
        telemetry.addData("LB Encoder", driveTrain.leftBack.getCurrentPosition());

        telemetry.update();
    }

    public static void outputDriveTelemetry(Telemetry telemetry, GamepadController.DriveType driveType,
                                            MecanumDrive drive){


    }
}
