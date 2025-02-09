package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="DC Motor Test OpMode", group="Tests")
public class DCMotorTestOpMode extends LinearOpMode {

    DcMotorEx testMotor;
    private final double MAX_VELOCITY = 2460.0;
    private final int RUNNING_VELOCITY = 2000;

    boolean gp1ButtonYLast = false;
    boolean gp1ButtonALast = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialization code here
        testMotor = hardwareMap.get(DcMotorEx.class, "testMotor");
        initMotor();

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        telemetry.addLine("Start Pressed");
        telemetry.update();

        // Run the loop while the opmode is active
        while (opModeIsActive()) {

            if ( isStopRequested())
                break;

            int currentPosition = testMotor.getCurrentPosition();
            telemetry.addData("Current Position", currentPosition);

            // Get the gamepad inputs

            if (gp1GetButtonYPress()) {
                telemetry.addLine("Gamepad 1 Y Pressed");
                runMotorToPosition(1000);
            } else if (gp1GetButtonAPress()) {
                telemetry.addLine("Gamepad 1 A Pressed");
                runMotorToPosition(0);
            }

//            double input = gamepad1.left_stick_y;
//            double velocity = input * MAX_VELOCITY;
//
//            telemetry.addData("Velocity", velocity);
//
//            // Set the power for the motor
//            testMotor.setVelocity(velocity);

            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }

    private void initMotor() {
        testMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //testMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        testMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Calculate PIDF values for velocity control
        double kF = 32767/MAX_VELOCITY;
        double kP = 0.1 * kF;
        double kI = 0.1 * kP;
        double kD = 0.0;

        testMotor.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        testMotor.setPositionPIDFCoefficients(5.0);
    }

    private void runMotorToPosition(int position) {

        telemetry.addData("Target Position", position);
        testMotor.setTargetPosition(position);
        testMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        testMotor.setVelocity(RUNNING_VELOCITY);
    }

    public boolean gp1GetButtonYPress() {
        boolean isPressedButtonY = false;
        if (!gp1ButtonYLast && gamepad1.y) {
            isPressedButtonY = true;
        }
        gp1ButtonYLast = gamepad1.y;
        return isPressedButtonY;
    }

    public boolean gp1GetButtonAPress() {
        boolean isPressedButtonA = false;
        if (!gp1ButtonALast && gamepad1.a) {
            isPressedButtonA = true;
        }
        gp1ButtonALast = gamepad1.a;
        return isPressedButtonA;
    }
}
