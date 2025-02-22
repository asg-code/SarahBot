package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;

/**
 * SimpleRobotOrientedDriveOpMode - A simple teleop opmode for a mecanum drive train
 * OpMode for a mecanum drive traiith robot oriented drive
 */
@TeleOp(name="Robot Oriented Drive OpMode", group="Tests")
public class RobotOrientedDriveOpMode extends LinearOpMode {
    // Initialize the mecanum drive train
    private MecanumDrive driveTrain;
    Vector2d gamepadInput;
    double gamepadInputTurn;

    @Override
    public void runOpMode() {

        driveTrain = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the start button to be pressed
        waitForStart();

        // Run the loop while the opmode is active
        while (opModeIsActive()) {
            // Get the gamepad inputs
            gamepadInputTurn = -gamepad1.right_stick_x;
            gamepadInput = new Vector2d(
                    -gamepad1.left_stick_y,
                    gamepad1.right_stick_x);

            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            telemetry.addData("Forward", forward);
            telemetry.addData("Strafe", strafe);
            telemetry.addData("Rotate", rotate);

            // Set the powers for the mecanum drive train
            driveTrain.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            gamepadInput.x,
                            gamepadInput.y),
                    gamepadInputTurn
            ));

            driveTrain.updatePoseEstimate();

            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}