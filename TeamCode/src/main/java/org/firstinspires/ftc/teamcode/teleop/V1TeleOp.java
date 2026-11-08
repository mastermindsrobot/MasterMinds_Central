
package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "V1TeleOp", group = "Competition")
public class V1TeleOp extends LinearOpMode {

    // Drivetrain motors
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // Mechanism motors
    private DcMotor intake;
    private DcMotor booster;
    private DcMotor flywheel;

    // Turret servo
    private Servo turret1;

    // IMU
    private IMU imu;

    // Intake state
    private boolean intakeRunning = false;
    private boolean previousLeftTrigger = false;

    // Turret position
    private double turretPosition = 0.5;

    // Turret adjustment increment
    private static final double TURRET_STEP = 0.0025;

    // Motor powers
    private static final double INTAKE_POWER = 1.0;
    private static final double BOOSTER_POWER = 1.0;
    private static final double FLYWHEEL_POWER = 1.0;

    // Drive speed
    private static final double DRIVE_SPEED = 1.0;

    @Override
    public void runOpMode() {

        // =========================================
        // HARDWARE INITIALIZATION
        // =========================================

        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backRight = hardwareMap.get(DcMotor.class, "back_right");

        intake = hardwareMap.get(DcMotor.class, "intake");
        booster = hardwareMap.get(DcMotor.class, "booster");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");

        turret1 = hardwareMap.get(Servo.class, "turret1");

        imu = hardwareMap.get(IMU.class, "imu");

        // =========================================
        // MOTOR CONFIGURATION
        // =========================================

        // Drivetrain directions
        // Verify these against your physical motor installation.

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Mechanism directions
        intake.setDirection(DcMotor.Direction.REVERSE);
        booster.setDirection(DcMotor.Direction.FORWARD);
        flywheel.setDirection(DcMotor.Direction.FORWARD);

        // Use encoder-free operation
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        booster.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Brake drivetrain motors when no power is applied
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // =========================================
        // TURRET INITIALIZATION
        // =========================================

        turretPosition = 0.5;
        turret1.setPosition(turretPosition);

        // =========================================
        // IMU CONFIGURATION
        // =========================================

        RevHubOrientationOnRobot orientation =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                );

        imu.initialize(
                new IMU.Parameters(orientation)
        );

        imu.resetYaw();

        // =========================================
        // TELEMETRY
        // =========================================

        telemetry.addLine("V1TeleOp Initialized");
        telemetry.addLine("Waiting for START...");
        telemetry.update();

        waitForStart();

        // =========================================
        // STARTUP
        // =========================================

        if (isStopRequested()) return;

        // Flywheel automatically starts
        flywheel.setPower(FLYWHEEL_POWER);

        // =========================================
        // MAIN TELEOP LOOP
        // =========================================

        while (opModeIsActive()) {

            // =====================================
            // 1. MECANUM DRIVETRAIN
            // =====================================

            // Left joystick: translation
            double y = -gamepad1.right_stick_y;
            double x = gamepad1.right_stick_x;

            // Right joystick X: rotation
            double rx = gamepad1.left_stick_x;

            // Normalize motor powers
            double denominator = Math.max(
                    Math.abs(y) + Math.abs(x) + Math.abs(rx),
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

            // Apply drive speed
            frontLeft.setPower(frontLeftPower * DRIVE_SPEED);
            backLeft.setPower(backLeftPower * DRIVE_SPEED);

            frontRight.setPower(frontRightPower * DRIVE_SPEED);
            backRight.setPower(backRightPower * DRIVE_SPEED);


            // =====================================
            // 2. INTAKE TOGGLE
            // =====================================

            boolean leftTriggerPressed =
                    gamepad1.left_trigger > 0.5;

            // Detect a new trigger press
            if (leftTriggerPressed && !previousLeftTrigger) {

                intakeRunning = !intakeRunning;

            }

            previousLeftTrigger = leftTriggerPressed;

            // =====================================
            // 3. INTAKE / OUTTAKE CONTROL
            // =====================================

            if (gamepad1.left_bumper) {

                // Hold left bumper to outtake
                intake.setPower(-INTAKE_POWER);

            } else if (intakeRunning) {

                // Return to intake when bumper released
                intake.setPower(INTAKE_POWER);

            } else {

                // Intake disabled
                intake.setPower(0);
            }


            // =====================================
            // 4. BOOSTER CONTROL
            // =====================================

            if (gamepad1.right_trigger > 0.5) {

                // Hold right trigger to activate booster
                booster.setPower(BOOSTER_POWER);

            } else {

                booster.setPower(0);
            }


            // =====================================
            // 5. FLYWHEEL CONTROL
            // =====================================

            // Automatically remains running during TeleOp
            flywheel.setPower(FLYWHEEL_POWER);


            // =====================================
            // 6. TURRET SERVO CONTROL
            // =====================================

            // D-pad left: rotate turret left
            if (gamepad1.dpad_left) {

                turretPosition -= TURRET_STEP;

            }

            // D-pad right: rotate turret right
            if (gamepad1.dpad_right) {

                turretPosition += TURRET_STEP;

            }

            // Prevent servo position from exceeding limits
            turretPosition = Math.max(
                    0.0,
                    Math.min(1.0, turretPosition)
            );

            turret1.setPosition(turretPosition);


            // =====================================
            // 7. IMU HEADING RESET
            // =====================================

            if (gamepad1.x) {

                imu.resetYaw();

            }

            double heading = imu.getRobotYawPitchRollAngles()
                    .getYaw(AngleUnit.DEGREES);


            // =====================================
            // 8. TELEMETRY
            // =====================================

            telemetry.addLine("===== V1 TELEOP =====");

            telemetry.addData("Heading", "%.2f degrees", heading);

            telemetry.addData("Intake",
                    intakeRunning ? "ON" : "OFF");

            telemetry.addData("Outtake Override",
                    gamepad1.left_bumper ? "ACTIVE" : "INACTIVE");

            telemetry.addData("Booster",
                    gamepad1.right_trigger > 0.5 ? "ON" : "OFF");

            telemetry.addData("Flywheel", "RUNNING");

            telemetry.addData("Turret Position",
                    "%.4f", turretPosition);

            telemetry.addData("Front Left", "%.2f", frontLeftPower);
            telemetry.addData("Front Right", "%.2f", frontRightPower);
            telemetry.addData("Back Left", "%.2f", backLeftPower);
            telemetry.addData("Back Right", "%.2f", backRightPower);

            telemetry.update();
        }

        // =========================================
        // STOP ALL MOTORS
        // =========================================

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        intake.setPower(0);
        booster.setPower(0);
        flywheel.setPower(0);
    }
}