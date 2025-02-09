package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name="Limit DC Motor Test OpMode", group="Tests")
public class LimitDCMotorTest extends LinearOpMode {
    private DcMotorEx testMotor;
    public TouchSensor limitSwitch;

    private final double POWER_LEVEL_STOP = 0.0;
    private final double POWER_LEVEL_RUN = 0.9;

    private final double MAX_VELOCITY = 2460.0;
    private final int RUNNING_VELOCITY = 2000;

    private final int MOTOR_LOW_POSITION = 0;
    private final int MOTOR_HIGH_POSITION = 1000;

    // Keep this small so we have an opportunity to stop the motor
    // if it is too big, then we will not have a chance to stop the motor because it will run
    // for a while before we can do the checks
    private final int MOTOR_DELTA = 100;

    boolean gp1ButtonYLast = false;
    boolean gp1ButtonALast = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialization code here
        testMotor = hardwareMap.get(DcMotorEx.class, "testMotor");
        limitSwitch = hardwareMap.get(TouchSensor.class, "limitSwitch");

        initMotor();

        waitForStart();

        telemetry.setAutoClear(true);
        telemetry.addLine("Start Pressed");
        telemetry.update();

        boolean runToLimitSwitch = false;

        while (opModeIsActive()) {
            // Loop code here
            if ( isStopRequested())
                break;

            int currentPosition = testMotor.getCurrentPosition();
            telemetry.addData("Current Position", currentPosition);
            telemetry.addData("Limit Switch", limitSwitch.isPressed());

            boolean isLimitSwitchPressed = limitSwitch.isPressed();
            boolean wantsToGoUp = gp1GetButtonYPress();
            boolean wantsToGoDown = gp1GetButtonAPress();
            boolean override = gamepad1.left_bumper;

            // If override is pressed, run the motor to the limit switch
            // this is sort of a state machine, every loop, we check if we
            // have to run the motor to the limit switch, and if yes, we do that
            if (override) {
                runToLimitSwitch = true;
            }

            if (runToLimitSwitch) {
                telemetry.addData("Status", "Running in override mode");
                telemetry.update();
                runToLimitSwitch = runMotorToLimitSwitch();
                continue;
            }

            // react to gamepad inputs
            if (wantsToGoUp) {
                runMotorToPosition(MOTOR_HIGH_POSITION);
            }
            // If we have to move down, use encoder to move down
            else if (wantsToGoDown && !isLimitSwitchPressed) {
                runMotorToPosition(MOTOR_LOW_POSITION);
            }

            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }

    private void initMotor() {
        testMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

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

    /**
     * Moves the motor down until the limit switch is pressed
     * Moves the motor down by delta each time (keep delta small)
     * @return false if the limit switch is pressed, true otherwise to
     * indicate that we have not hit the limit switch
     */
    private boolean runMotorToLimitSwitch() {
        // Move the motor down until the limit switch is pressed
        // Here we are switching to power mode instead of velocity mode
        // because we really don't care about PID in this override case - just bring the motor down
        // also, LEDs are cool, but we are not using them here
        boolean isLimitSwitchPressed = limitSwitch.isPressed();

        // If limit switch is pressed, stop the motor and return false to get out of override mode
        if (isLimitSwitchPressed) {
            stopMotor();
            return false;
        }

        // otherwise, keep moving the motor down by delta
        int position = testMotor.getCurrentPosition();
        position -= MOTOR_DELTA;
        testMotor.setTargetPosition(position);
        testMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        testMotor.setPower(POWER_LEVEL_RUN);

        // return true to indicate that we have not hit the limit switch
        return true;
    }

    private void stopMotor() {
        testMotor.setPower(POWER_LEVEL_STOP);
        testMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    private boolean gp1GetButtonYPress() {
        boolean isPressedButtonY = false;
        if (!gp1ButtonYLast && gamepad1.y) {
            isPressedButtonY = true;
        }
        gp1ButtonYLast = gamepad1.y;
        return isPressedButtonY;
    }

    private boolean gp1GetButtonAPress() {
        boolean isPressedButtonA = false;
        if (!gp1ButtonALast && gamepad1.a) {
            isPressedButtonA = true;
        }
        gp1ButtonALast = gamepad1.a;
        return isPressedButtonA;
    }
}
