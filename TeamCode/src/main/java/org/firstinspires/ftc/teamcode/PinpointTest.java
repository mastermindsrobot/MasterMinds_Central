package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Pinpoint Odometry Test", group = "Odometry")
public class PinpointTest extends LinearOpMode {

    private GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() {

        // -------------------------
        // MAP PINPOINT
        // -------------------------

        pinpoint = hardwareMap.get(
                GoBildaPinpointDriver.class,
                "pinpoint"
        );

        // -------------------------
        // ENCODER RESOLUTION
        // -------------------------

        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );

        // -------------------------
        // ENCODER DIRECTIONS
        // -------------------------

        /*
         * Your setup:
         *
         * Pinpoint X encoder -> parallel pod
         * Pinpoint Y encoder -> perpendicular pod
         *
         * Start with both FORWARD.
         */

        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        // -------------------------
        // RESET POSITION
        // -------------------------

        /*
         * Keep the robot completely stationary.
         *
         * This establishes:
         *
         * X = 0
         * Y = 0
         * Heading = 0
         */

        pinpoint.resetPosAndIMU();

        telemetry.addLine("PINPOINT READY");
        telemetry.addLine("");
        telemetry.addLine("X: 0.00 in");
        telemetry.addLine("Y: 0.00 in");
        telemetry.addLine("Heading: 0.00 deg");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        // -------------------------
        // MAIN LOOP
        // -------------------------

        while (opModeIsActive()) {

            // Update Pinpoint
            pinpoint.update();

            // -------------------------
            // POSITION
            // -------------------------

            double posX = pinpoint.getPosX(
                    DistanceUnit.INCH
            );

            double posY = pinpoint.getPosY(
                    DistanceUnit.INCH
            );

            double heading = pinpoint.getHeading(
                    AngleUnit.DEGREES
            );

            // -------------------------
            // VELOCITY
            // -------------------------

            double velocityX = pinpoint.getVelX(
                    DistanceUnit.INCH
            );

            double velocityY = pinpoint.getVelY(
                    DistanceUnit.INCH
            );

            // -------------------------
            // TELEMETRY
            // -------------------------

            telemetry.addLine("========== PINPOINT ==========");

            telemetry.addData(
                    "X Position",
                    "%.2f in",
                    posX
            );

            telemetry.addData(
                    "Y Position",
                    "%.2f in",
                    posY
            );

            telemetry.addData(
                    "Heading",
                    "%.2f deg",
                    heading
            );

            telemetry.addLine("");

            telemetry.addLine("========== VELOCITY ==========");

            telemetry.addData(
                    "X Velocity",
                    "%.2f in/s",
                    velocityX
            );

            telemetry.addData(
                    "Y Velocity",
                    "%.2f in/s",
                    velocityY
            );

            telemetry.update();
        }
    }
}