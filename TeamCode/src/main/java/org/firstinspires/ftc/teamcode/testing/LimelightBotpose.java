package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp(name = "Limelight BotPose Test", group = "Vision")
public class LimelightBotpose extends LinearOpMode {

    private Limelight3A limelight;

    // Limelight reports distance in meters.
    // Pedro Pathing uses inches.
    private static final double METERS_TO_INCHES = 39.3700787;

    @Override
    public void runOpMode() throws InterruptedException {

        // ==========================================
        // LIMELIGHT
        // ==========================================

        limelight = hardwareMap.get(
                Limelight3A.class,
                "limelight"
        );

        // Select AprilTag pipeline
        limelight.pipelineSwitch(0);

        // Start receiving Limelight data
        limelight.start();

        telemetry.addLine("Limelight initialized");
        telemetry.addLine("Pipeline 0");
        telemetry.addLine("Distances will be shown in inches");
        telemetry.addLine("Press PLAY");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            telemetry.clearAll();

            // ==========================================
            // STATUS
            // ==========================================

            LLStatus status = limelight.getStatus();

            telemetry.addLine("===== LIMELIGHT STATUS =====");

            telemetry.addData(
                    "Name",
                    status.getName()
            );

            telemetry.addData(
                    "FPS",
                    "%.1f",
                    status.getFps()
            );

            telemetry.addData(
                    "Pipeline",
                    "%d / %s",
                    status.getPipelineIndex(),
                    status.getPipelineType()
            );

            telemetry.addData(
                    "CPU",
                    "%.1f%%",
                    status.getCpu()
            );

            telemetry.addData(
                    "Temperature",
                    "%.1f C",
                    status.getTemp()
            );

            // ==========================================
            // RESULT
            // ==========================================

            LLResult result =
                    limelight.getLatestResult();

            telemetry.addLine("");
            telemetry.addLine("===== RESULT =====");

            telemetry.addData(
                    "Valid",
                    result.isValid()
            );

            if (result.isValid()) {

                // ==========================================
                // LATENCY
                // ==========================================

                telemetry.addData(
                        "Capture Latency",
                        "%.1f ms",
                        result.getCaptureLatency()
                );

                telemetry.addData(
                        "Targeting Latency",
                        "%.1f ms",
                        result.getTargetingLatency()
                );

                telemetry.addData(
                        "Parse Latency",
                        "%.1f ms",
                        result.getParseLatency()
                );

                // ==========================================
                // TX / TY
                // ==========================================

                telemetry.addLine("");
                telemetry.addLine("===== TARGET =====");

                telemetry.addData(
                        "TX",
                        "%.2f°",
                        result.getTx()
                );

                telemetry.addData(
                        "TY",
                        "%.2f°",
                        result.getTy()
                );

                // ==========================================
                // APRILTAGS
                // ==========================================

                List<LLResultTypes.FiducialResult> tags =
                        result.getFiducialResults();

                telemetry.addData(
                        "AprilTags",
                        tags.size()
                );

                for (int i = 0; i < tags.size(); i++) {

                    LLResultTypes.FiducialResult tag =
                            tags.get(i);

                    telemetry.addData(
                            "Tag " + i,
                            "ID=%d  Family=%s",
                            tag.getFiducialId(),
                            tag.getFamily()
                    );

                    telemetry.addData(
                            "Tag " + i + " TX/TY",
                            "%.2f° / %.2f°",
                            tag.getTargetXDegrees(),
                            tag.getTargetYDegrees()
                    );
                }

                // ==========================================
                // BOTPOSE
                // ==========================================

                Pose3D botpose =
                        result.getBotpose();

                telemetry.addLine("");
                telemetry.addLine("===== BOTPOSE =====");

                if (botpose != null) {

                    // ------------------------------------------
                    // RAW VALUES FROM LIMELIGHT
                    // ------------------------------------------

                    double xMeters =
                            botpose.getPosition().x;

                    double yMeters =
                            botpose.getPosition().y;

                    double zMeters =
                            botpose.getPosition().z;

                    // ------------------------------------------
                    // CONVERT METERS → INCHES
                    // ------------------------------------------

                    double xInches =
                            xMeters * METERS_TO_INCHES;

                    double yInches =
                            yMeters * METERS_TO_INCHES;

                    double zInches =
                            zMeters * METERS_TO_INCHES;

                    // ------------------------------------------
                    // ROTATION
                    // ------------------------------------------

                    double yaw =
                            botpose
                                    .getOrientation()
                                    .getYaw(
                                            AngleUnit.DEGREES
                                    );

                    double pitch =
                            botpose
                                    .getOrientation()
                                    .getPitch(
                                            AngleUnit.DEGREES
                                    );

                    double roll =
                            botpose
                                    .getOrientation()
                                    .getRoll(
                                            AngleUnit.DEGREES
                                    );

                    // ------------------------------------------
                    // PEDRO-STYLE COORDINATES
                    // ------------------------------------------

                    telemetry.addData(
                            "X",
                            "%.2f in",
                            xInches
                    );

                    telemetry.addData(
                            "Y",
                            "%.2f in",
                            yInches
                    );

                    telemetry.addData(
                            "Z",
                            "%.2f in",
                            zInches
                    );

                    telemetry.addData(
                            "Heading / Yaw",
                            "%.2f°",
                            yaw
                    );

                    // ------------------------------------------
                    // ORIENTATION
                    // ------------------------------------------

                    telemetry.addData(
                            "Pitch",
                            "%.2f°",
                            pitch
                    );

                    telemetry.addData(
                            "Roll",
                            "%.2f°",
                            roll
                    );

                    // ------------------------------------------
                    // RAW VALUES
                    // ------------------------------------------

                    telemetry.addLine("");
                    telemetry.addLine("===== RAW LIMELIGHT =====");

                    telemetry.addData(
                            "X Raw",
                            "%.4f m",
                            xMeters
                    );

                    telemetry.addData(
                            "Y Raw",
                            "%.4f m",
                            yMeters
                    );

                    telemetry.addData(
                            "Z Raw",
                            "%.4f m",
                            zMeters
                    );

                } else {

                    telemetry.addLine(
                            "BotPose = NULL"
                    );
                }

            } else {

                telemetry.addLine("");
                telemetry.addLine(
                        "No valid Limelight result"
                );
            }

            telemetry.update();

            sleep(20);
        }

        // ==========================================
        // STOP LIMELIGHT
        // ==========================================

        limelight.stop();
    }
}