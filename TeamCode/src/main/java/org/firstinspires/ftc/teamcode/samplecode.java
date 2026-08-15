package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Mecanum Drive with Brakes", group = "TeleOp")
public class samplecode extends LinearOpMode {

    private DcMotor leftFront, rightFront, leftBack, rightBack;
    private GoBildaPinpointDriver pinpoint;

    // Starts the program in slow mode
    private boolean speedToggle = false;

    // Remembers if left bumper was pressed during last cycle
    private boolean leftBumperWasPressed = false;

    @Override
    public void runOpMode() {

        // -------------------------
        // MAP MOTORS
        // -------------------------

        leftFront = hardwareMap.get(DcMotor.class, "front-left");
        rightFront = hardwareMap.get(DcMotor.class, "front-right");
        leftBack = hardwareMap.get(DcMotor.class, "back-left");
        rightBack = hardwareMap.get(DcMotor.class, "back-right");

        // -------------------------
        // MAP PINPOINT
        // -------------------------

        pinpoint = hardwareMap.get(
                GoBildaPinpointDriver.class,
                "pinpoint"
        );

        // -------------------------
        // MOTOR DIRECTIONS
        // -------------------------

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // -------------------------
        // MOTOR MODES
        // -------------------------

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // -------------------------
        // RESET PINPOINT
        // -------------------------

        // Robot should be completely stationary here.
        // This makes the current orientation 0 degrees.
        pinpoint.resetPosAndIMU();

        telemetry.addData("Status", "Ready");
        telemetry.addData("Pinpoint", "Initialized");
        telemetry.update();

        // -------------------------
        // WAIT FOR START
        // -------------------------

        waitForStart();

        // -------------------------
        // MAIN LOOP
        // -------------------------

        while (opModeIsActive()) {

            // Update Pinpoint
            pinpoint.update();

            // Get robot heading in radians
            double heading = pinpoint.getHeading(AngleUnit.RADIANS);


            // -------------------------
            // SPEED TOGGLE
            // -------------------------

            boolean leftBumperNow = gamepad1.left_bumper;

            // Detect a new button press
            if (leftBumperNow && !leftBumperWasPressed) {
                speedToggle = !speedToggle;
            }

            leftBumperWasPressed = leftBumperNow;

            // Slow = 0.5
            // Fast = 1.0
            double speedMultiplier = speedToggle ? 1.0 : 0.5;

            // -------------------------
            // GAMEPAD INPUT
            // -------------------------

            // FTC joystick Y is negative when pushed forward,
            // so we invert it here.
            double y = gamepad1.right_stick_y;

            // X controls strafing
            double x = -gamepad1.right_stick_x;

            // Left stick controls rotation
            double rx = -gamepad1.left_stick_x;

            // -------------------------
            // FIELD-CENTRIC TRANSFORM
            // -------------------------

            /*
             * Rotate the joystick vector by the robot's
             * current heading.
             *
             * This converts robot-centric joystick input
             * into field-centric input.
             */

            double rotX = x * Math.cos(heading)
                    - y * Math.sin(heading);

            double rotY = x * Math.sin(heading)
                    + y * Math.cos(heading);

            // Slight compensation for mecanum strafing
            rotX *= 1.1;

            // -------------------------
            // MECANUM CALCULATION
            // -------------------------

            double denominator = Math.max(
                    Math.abs(rotY)
                            + Math.abs(rotX)
                            + Math.abs(rx),
                    1.0
            );

            double frontLeftPower =
                    (rotY + rotX + rx) / denominator;

            double backLeftPower =
                    (rotY - rotX + rx) / denominator;

            double frontRightPower =
                    (rotY - rotX - rx) / denominator;

            double backRightPower =
                    (rotY + rotX - rx) / denominator;

            // -------------------------
            // APPLY MOTOR POWER
            // -------------------------

            leftFront.setPower(frontLeftPower * speedMultiplier);
            leftBack.setPower(backLeftPower * speedMultiplier);

            rightFront.setPower(frontRightPower * speedMultiplier);
            rightBack.setPower(backRightPower * speedMultiplier);

            // -------------------------
            // TELEMETRY
            // -------------------------

            telemetry.addData(
                    "Heading",
                    "%.1f degrees",
                    Math.toDegrees(heading)
            );

            telemetry.addData(
                    "Speed Mode",
                    speedToggle ? "Fast" : "Slow"
            );

            telemetry.addData(
                    "FL Power",
                    "%.2f",
                    frontLeftPower * speedMultiplier
            );

            telemetry.addData(
                    "BL Power",
                    "%.2f",
                    backLeftPower * speedMultiplier
            );

            telemetry.addData(
                    "FR Power",
                    "%.2f",
                    frontRightPower * speedMultiplier
            );

            telemetry.addData(
                    "BR Power",
                    "%.2f",
                    backRightPower * speedMultiplier
            );

            telemetry.update();
        }
    }
}