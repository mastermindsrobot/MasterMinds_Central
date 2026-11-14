
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "MasterMinds Demo TeleOp", group = "MasterMinds")
public class DemoTeleOp extends LinearOpMode {

    // Drivetrain motors
    private DcMotor frontLeft;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    // Mechanism motors
    private DcMotor intake;
    private DcMotor booster;
    private DcMotor flywheel;

    // Control Hub IMU
    private IMU imu;

    // Mechanism states
    private boolean shooterOn = false;
    private boolean intakeOn = false;

    // Button edge detection
    private boolean previousLeftBumper = false;
    private boolean previousRightBumper = false;
    private boolean previousX = false;

    // Motor powers
    private static final double FLYWHEEL_POWER = 0.8;
    private static final double INTAKE_POWER = 1.0;
    private static final double BOOSTER_POWER = 1.0;

    // Drivetrain speed
    private static final double SPEED_MULTIPLIER = 0.8;

    // Strafe compensation
    private static final double STRAFE_MULTIPLIER = 1.1;

    @Override
    public void runOpMode() {

        // =========================================
        // 1. MAP MOTORS
        // =========================================

        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        rightFront = hardwareMap.get(DcMotor.class, "front_right");
        leftBack = hardwareMap.get(DcMotor.class, "back_left");
        rightBack = hardwareMap.get(DcMotor.class, "back_right");

        intake = hardwareMap.get(DcMotor.class, "intake");
        booster = hardwareMap.get(DcMotor.class, "booster");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");

        // =========================================
        // 2. DRIVETRAIN DIRECTIONS
        // Preserved from your working MecanumDrive
        // =========================================

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        // Reverse intake and booster
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        booster.setDirection(DcMotorSimple.Direction.REVERSE);

        // =========================================
        // 3. MOTOR MODES
        // =========================================

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initial motor powers
        frontLeft.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        intake.setPower(0);
        booster.setPower(0);
        flywheel.setPower(0);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        booster.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // =========================================
        // 4. INITIALIZE CONTROL HUB IMU
        // Assumed mounting:
        // REV logo faces LEFT
        // USB ports face FORWARD
        // =========================================

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot hubOrientation =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                );

        imu.initialize(new IMU.Parameters(hubOrientation));

        telemetry.addLine("MasterMinds TeleOp Ready");
        telemetry.addLine("Field-Centric Mecanum Drive");
        telemetry.addLine("Press X to reset heading");
        telemetry.addData("IMU Logo Direction", "LEFT");
        telemetry.addData("IMU USB Direction", "FORWARD");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        // Make the robot's current direction the initial forward.
        imu.resetYaw();

        // =========================================
        // 5. MAIN LOOP
        // =========================================

        while (opModeIsActive()) {

            // =========================================
            // FIELD-CENTRIC MECANUM DRIVE
            // Right stick Y = forward/backward
            // Right stick X = strafe left/right
            // Left stick X  = rotate
            // =========================================

            // Field-relative joystick inputs
            double y = -gamepad1.right_stick_y;
            double x = gamepad1.right_stick_x;

            // Rotation remains robot-relative
            double rx = gamepad1.left_stick_x;

            // =========================================
            // RESET HEADING WITH X
            // The current direction becomes field forward
            // =========================================

            boolean xPressed = gamepad1.x;

            if (xPressed && !previousX) {
                imu.resetYaw();
            }

            previousX = xPressed;

            // =========================================
            // READ IMU HEADING
            // =========================================

            YawPitchRollAngles angles =
                    imu.getRobotYawPitchRollAngles();

            double heading = angles.getYaw(AngleUnit.RADIANS);

            // =========================================
            // FIELD-TO-ROBOT COORDINATE TRANSFORMATION
            // =========================================

            double cosHeading = Math.cos(-heading);
            double sinHeading = Math.sin(-heading);

            double rotatedX =
                    x * cosHeading - y * sinHeading;

            double rotatedY =
                    x * sinHeading + y * cosHeading;

            // Strafe compensation
            rotatedX *= STRAFE_MULTIPLIER;

            // =========================================
            // MECANUM POWER CALCULATION
            // =========================================

            double denominator = Math.max(
                    Math.abs(rotatedY)
                            + Math.abs(rotatedX)
                            + Math.abs(rx),
                    1.0
            );

            double frontLeftPower =
                    (rotatedY + rotatedX + rx) / denominator;

            double backLeftPower =
                    (rotatedY - rotatedX + rx) / denominator;

            double frontRightPower =
                    (rotatedY - rotatedX - rx) / denominator;

            double backRightPower =
                    (rotatedY + rotatedX - rx) / denominator;

            // Apply drivetrain power
            frontLeft.setPower(
                    frontLeftPower * SPEED_MULTIPLIER
            );

            leftBack.setPower(
                    backLeftPower * SPEED_MULTIPLIER
            );

            rightFront.setPower(
                    frontRightPower * SPEED_MULTIPLIER
            );

            rightBack.setPower(
                    backRightPower * SPEED_MULTIPLIER
            );

            // =========================================
            // FLYWHEEL TOGGLE
            // Left bumper: ON/OFF at 0.8 power
            // =========================================

            boolean leftBumperPressed = gamepad1.left_bumper;

            if (leftBumperPressed && !previousLeftBumper) {
                shooterOn = !shooterOn;
            }

            previousLeftBumper = leftBumperPressed;

            flywheel.setPower(
                    shooterOn ? FLYWHEEL_POWER : 0.0
            );

            // =========================================
            // INTAKE TOGGLE
            // Right bumper: ON/OFF
            // =========================================

            boolean rightBumperPressed = gamepad1.right_bumper;

            if (rightBumperPressed && !previousRightBumper) {
                intakeOn = !intakeOn;
            }

            previousRightBumper = rightBumperPressed;

            // =========================================
            // BOOSTER
            // Right trigger: run while held
            // Intake must run whenever booster runs
            // =========================================

            boolean boosterOn = gamepad1.right_trigger > 0.1;

            boolean actualIntakeOn = intakeOn || boosterOn;

            intake.setPower(
                    actualIntakeOn ? INTAKE_POWER : 0.0
            );

            if (boosterOn) {
                double triggerPower = Range.clip(
                        gamepad1.right_trigger,
                        0.0,
                        1.0
                );

                booster.setPower(
                        BOOSTER_POWER * triggerPower
                );
            } else {
                booster.setPower(0.0);
            }

            // =========================================
            // TELEMETRY
            // =========================================

            telemetry.addData("Drive Mode", "FIELD-CENTRIC");
            telemetry.addData(
                    "Heading",
                    "%.1f degrees",
                    angles.getYaw(AngleUnit.DEGREES)
            );

            telemetry.addData(
                    "Drive Speed",
                    "%.0f%%",
                    SPEED_MULTIPLIER * 100
            );

            telemetry.addData(
                    "FL Power", "%.2f",
                    frontLeftPower * SPEED_MULTIPLIER
            );

            telemetry.addData(
                    "BL Power", "%.2f",
                    backLeftPower * SPEED_MULTIPLIER
            );

            telemetry.addData(
                    "FR Power", "%.2f",
                    frontRightPower * SPEED_MULTIPLIER
            );

            telemetry.addData(
                    "BR Power", "%.2f",
                    backRightPower * SPEED_MULTIPLIER
            );

            telemetry.addData(
                    "Flywheel",
                    shooterOn ? "ON" : "OFF"
            );

            telemetry.addData(
                    "Flywheel Power",
                    shooterOn ? FLYWHEEL_POWER : 0.0
            );

            telemetry.addData(
                    "Intake Toggle",
                    intakeOn ? "ON" : "OFF"
            );

            telemetry.addData(
                    "Actual Intake",
                    actualIntakeOn ? "ON" : "OFF"
            );

            telemetry.addData(
                    "Booster",
                    boosterOn ? "ON" : "OFF"
            );

            telemetry.addLine("Press X to reset field heading");

            telemetry.update();
        }
    }
}