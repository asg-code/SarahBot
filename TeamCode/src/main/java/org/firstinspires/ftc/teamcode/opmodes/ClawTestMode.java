package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Claw Test OpMode", group="Tests")
public class ClawTestMode extends LinearOpMode {

    Servo clawServo;
    Servo wristServo;

    private final double CLAW_CLOSED = 0.22;
    private final double CLAW_OPEN = 0.0;

    private final double WRIST_DOWN = 0.8;
    private final double WRIST_UP = 0.1;

    @Override
    public void runOpMode() throws InterruptedException {
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        wristServo = hardwareMap.get(Servo.class, "wristServo");

        clawServo.setPosition(CLAW_OPEN);
        wristServo.setPosition(WRIST_UP);

        waitForStart();

        while (opModeIsActive()) {
            if (isStopRequested())
                break;



            if (gamepad1.a) {
                clawServo.setPosition(CLAW_CLOSED);
                telemetry.addLine("Claw Closed");
            } else if (gamepad1.b) {
                clawServo.setPosition(CLAW_OPEN);
                telemetry.addLine("Claw Open");
            }
            else if (gamepad1.x) {
                wristServo.setPosition(WRIST_DOWN);
                telemetry.addLine("Wrist Down");
            } else if (gamepad1.y) {
                wristServo.setPosition(WRIST_UP);
                telemetry.addLine("Wrist Up");
            }

            telemetry.addData("Current Claw Servo position", clawServo.getPosition());
            telemetry.addData("Current Wrist Servo position", wristServo.getPosition());

            telemetry.update();

            idle();
        }
    }
}
