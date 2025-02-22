/*
Copyright (c) 2024 Limelight Vision

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.opmodes;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/*
 * This OpMode illustrates how to use the Limelight3A Vision Sensor.
 *
 * @see <a href="https://limelightvision.io/">Limelight</a>
 *
 * Notes on configuration:
 *
 *   The device presents itself, when plugged into a USB port on a Control Hub as an ethernet
 *   interface.  A DHCP server running on the Limelight automatically assigns the Control Hub an
 *   ip address for the new ethernet interface.
 *
 *   Since the Limelight is plugged into a USB port, it will be listed on the top level configuration
 *   activity along with the Control Hub Portal and other USB devices such as webcams.  Typically
 *   serial numbers are displayed below the device's names.  In the case of the Limelight device, the
 *   Control Hub's assigned ip address for that ethernet interface is used as the "serial number".
 *
 *   Tapping the Limelight's name, transitions to a new screen where the user can rename the Limelight
 *   and specify the Limelight's ip address.  Users should take care not to confuse the ip address of
 *   the Limelight itself, which can be configured through the Limelight settings page via a web browser,
 *   and the ip address the Limelight device assigned the Control Hub and which is displayed in small text
 *   below the name of the Limelight on the top level configuration screen.
 */
@TeleOp(name = "Limelight Track", group = "Sensor")

public class LimelightTrack extends LinearOpMode {

    private class DetectionInfo {
        public Pose3D botPose;
        public int tagId;

        public DetectionInfo(Pose3D botPose, int tagId) {
            this.botPose = botPose;
            this.tagId = tagId;
        }

    }

    private Map<Integer, Pose2d> tagLocations = new HashMap<>();
    private static final double CAMERA_Y = 5.0;
    private Limelight3A limelight;
    private MecanumDrive driveTrain;

    public LimelightTrack() {
        // Initialize the dictionary of tag locations.
        tagLocations.put(11, new Pose2d(-72, 48, 90));
        tagLocations.put(12, new Pose2d(-0, 72,0));
        tagLocations.put(13, new Pose2d(72, 48,  270));
        tagLocations.put(14, new Pose2d(72, -48, 270));
        tagLocations.put(15, new Pose2d(0, -72, 180));
        tagLocations.put(16, new Pose2d(-72, -40, 90));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        waitForStart();

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();

        LLStatus status = limelight.getStatus();
        telemetry.addData("Name", "%s",
                status.getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(), (int) status.getFps());
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());
        telemetry.update();

        DetectionInfo detectionInfo = null;
        Pose2d startPose = null;
        Pose2d targetPose = null;

        while (detectionInfo == null || isStopRequested()) {
            detectionInfo = getNormalizedRobotPose();

            if (detectionInfo != null && detectionInfo.botPose != null) {
                startPose = new Pose2d(detectionInfo.botPose.getPosition().x,
                        detectionInfo.botPose.getPosition().y,
                        detectionInfo.botPose.getOrientation().getYaw());

                targetPose = tagLocations.get(detectionInfo.tagId);
            }
        }

        telemetry.addData("Botpose (inches)",
                "X: %.2f, Y: %.2f, Z: %.2f Yaw: %.2f",
                detectionInfo.botPose.getPosition().x,
                detectionInfo.botPose.getPosition().y,
                detectionInfo.botPose.getPosition().z,
                detectionInfo.botPose.getOrientation().getYaw());

        // Initialize drive train
        driveTrain = new MecanumDrive(hardwareMap, startPose);
        telemetry.addData("DriveTrain Initialized with Pose:", startPose);
        telemetry.addData("Taget pose:", targetPose);
        telemetry.addLine("Press A to start tracking");
        telemetry.update();



        while (opModeIsActive()) {

            if (gamepad1.a) {

                Actions.runBlocking(
                        new SequentialAction(
                                driveToTag(startPose, targetPose, 0)
                        ));

                break;
            }

        }



//            LLResult result = limelight.getLatestResult();
//            if (result != null) {
//                // Access general information
//                Pose3D botpose = result.getBotpose();
//
//                if (result.isValid()) {
//
//                    // Convert botpose values from meters to inches
//                    double botposeXInches = botpose.getPosition().x * 39.3701;
//                    double botposeYInches = botpose.getPosition().y * 39.3701;
//                    double botposeZInches = botpose.getPosition().z * 39.3701;
//
//                    telemetry.addData("Botpose (inches)",
//                            "X: %.2f, Y: %.2f, Z: %.2f",
//                            botposeXInches, botposeYInches, botposeZInches);
//
//                    // Access fiducial results
//                    List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
//                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
//                        telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(),fr.getTargetXDegrees(), fr.getTargetYDegrees());
//                    }
//                }
//            } else {
//                telemetry.addData("Limelight", "No data available");
//            }



        limelight.stop();
    }

    private Action driveToTag(Pose2d startPose, Pose2d targetPose, double targetYaw) {
        TrajectoryActionBuilder tab = driveTrain.actionBuilder(startPose)
                .splineToLinearHeading(targetPose, targetYaw);

        return tab.build();
    }

    private DetectionInfo getNormalizedRobotPose() {
        LLResult result = limelight.getLatestResult();
        int tagId = -1;

        if (result != null) {
            tagId = result.getFiducialResults().get(0).getFiducialId();
            List<Double> samplesX = new ArrayList<>();
            List<Double> samplesY = new ArrayList<>();
            List<Double> samplesZ = new ArrayList<>();
            List<Double> sampleYaw = new ArrayList<>();

            for (int i = 0; i < 50; i++) {
                result = limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    tagId = result.getFiducialResults().get(0).getFiducialId();
                    Pose3D botpose = result.getBotpose();
                    samplesX.add(botpose.getPosition().x * 39.3701);
                    samplesY.add(botpose.getPosition().y * 39.3701-CAMERA_Y);
                    samplesZ.add(botpose.getPosition().z * 39.3701);
                    sampleYaw.add(botpose.getOrientation().getYaw());
                }

                sleep(5);
            }

            double meanX = calculateMean(samplesX);
            double meanY = calculateMean(samplesY);
            double meanZ = calculateMean(samplesZ);
            double meanYaw = calculateMean(sampleYaw);

            double stdDevX = calculateStandardDeviation(samplesX, meanX);
            double stdDevY = calculateStandardDeviation(samplesY, meanY);
            double stdDevZ = calculateStandardDeviation(samplesZ, meanZ);
            double stdDevYaw = calculateStandardDeviation(sampleYaw, meanYaw);

            List<Double> filteredX = filterOutliers(samplesX, meanX, stdDevX);
            List<Double> filteredY = filterOutliers(samplesY, meanY, stdDevY);
            List<Double> filteredZ = filterOutliers(samplesZ, meanZ, stdDevZ);
            List<Double> filteredYaw = filterOutliers(sampleYaw, meanYaw, stdDevYaw);

            double finalMeanX = calculateMean(filteredX);
            double finalMeanY = calculateMean(filteredY);
            double finalMeanZ = calculateMean(filteredZ);
            double finalMeanYaw = calculateMean(filteredYaw);

            Position pos = new  Position(DistanceUnit.INCH, finalMeanX, finalMeanY, finalMeanZ, 0);
            Pose3D pos2d = new Pose3D(pos, new YawPitchRollAngles(AngleUnit.DEGREES, finalMeanYaw, 0,0,0));

            return new DetectionInfo(pos2d, tagId);
        }

        return null;
    }

        private double calculateMean(List<Double> values) {
            double sum = 0.0;
            for (double value : values) {
                sum += value;
            }
            return sum / values.size();
        }

        private double calculateStandardDeviation(List<Double> values, double mean) {
            double sum = 0.0;
            for (double value : values) {
                sum += Math.pow(value - mean, 2);
            }
            return Math.sqrt(sum / values.size());
        }

        private List<Double> filterOutliers(List<Double> values, double mean, double stdDev) {
            List<Double> filteredValues = new ArrayList<>();
            for (double value : values) {
                if (Math.abs(value - mean) <= 2 * stdDev) {
                    filteredValues.add(value);
                }
            }
            return filteredValues;
        }

    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
            //don't even worry about it
        }
    }
}
