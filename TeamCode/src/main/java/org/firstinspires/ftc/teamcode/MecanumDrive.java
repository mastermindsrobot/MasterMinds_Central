package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Mecanum Drive", group = "TeleOp")
public class MecanumDrive extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    // false = slow mode
    // true = fast mode
    private boolean speedToggle = false;

    // Used to detect a new left bumper press
    private boolean leftBumperWasPressed = false;

    @Override
    public void runOpMode() {

        // =========================
        // MAP MOTORS
        // =========================

        leftFront = hardwareMap.get(
                DcMotor.class,
                "front-left"
        );

        rightFront = hardwareMap.get(
                DcMotor.class,
                "front-right"
        );

        leftBack = hardwareMap.get(
                DcMotor.class,
                "back-left"
        );

        rightBack = hardwareMap.get(
                DcMotor.class,
                "back-right"
        );

        // =========================
        // MOTOR DIRECTIONS
        // =========================

        leftFront.setDirection(
                DcMotorSimple.Direction.REVERSE
        );

        leftBack.setDirection(
                DcMotorSimple.Direction.REVERSE
        );

        rightFront.setDirection(
                DcMotorSimple.Direction.FORWARD
        );

        rightBack.setDirection(
                DcMotorSimple.Direction.FORWARD
        );

        // =========================
        // MOTOR MODES
        // =========================

        leftFront.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        rightFront.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        leftBack.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        rightBack.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        // =========================
        // READY
        // =========================

        telemetry.addLine("Mecanum Drive Ready");
        telemetry.addLine("Robot-Centric Mode");
        telemetry.update();

        // =========================
        // WAIT FOR START
        // =========================

        waitForStart();

        // =========================
        // MAIN LOOP
        // =========================

        while (opModeIsActive()) {

            // =========================
            // SPEED TOGGLE
            // =========================

            boolean leftBumperNow = gamepad1.left_bumper;

            // Toggle speed only once per button press
            if (leftBumperNow && !leftBumperWasPressed) {
                speedToggle = !speedToggle;
            }

            leftBumperWasPressed = leftBumperNow;

            // Slow = 50%
            // Fast = 100%
            double speedMultiplier =
                    speedToggle ? 0.8 : 0.5;

            // =========================
            // GAMEPAD INPUT
            // =========================

            // Forward / backward
            double y = -gamepad1.right_stick_y;

            // Strafing
            double x = gamepad1.right_stick_x;

            // Rotation
            double rx = gamepad1.left_stick_x;

            // =========================
            // STRAFE COMPENSATION
            // =========================

            x *= 1.1;

            // =========================
            // MECANUM CALCULATION
            // =========================

            double denominator = Math.max(
                    Math.abs(y)
                            + Math.abs(x)
                            + Math.abs(rx),
                    1.0
            );

            double frontLeftPower =
                    (y + x + rx) / denominator;

            double backLeftPower =
                    (y - x + rx) / denominator;

            double frontRightPower =
                    (y - x - rx) / denominator;

            double backRightPower =
                    (y + x - rx) / denominator;

            // =========================
            // APPLY MOTOR POWER
            // =========================

            leftFront.setPower(
                    frontLeftPower * speedMultiplier
            );

            leftBack.setPower(
                    backLeftPower * speedMultiplier
            );

            rightFront.setPower(
                    frontRightPower * speedMultiplier
            );

            rightBack.setPower(
                    backRightPower * speedMultiplier
            );

            // =========================
            // TELEMETRY
            // =========================

            telemetry.addData(
                    "Mode",
                    "Robot-Centric Mecanum"
            );

            telemetry.addData(
                    "Speed",
                    speedToggle ? "FAST" : "SLOW"
            );

            telemetry.addData(
                    "FL",
                    "%.2f",
                    frontLeftPower * speedMultiplier
            );

            telemetry.addData(
                    "BL",
                    "%.2f",
                    backLeftPower * speedMultiplier
            );

            telemetry.addData(
                    "FR",
                    "%.2f",
                    frontRightPower * speedMultiplier
            );

            telemetry.addData(
                    "BR",
                    "%.2f",
                    backRightPower * speedMultiplier
            );

            telemetry.update();
        }
    }
}