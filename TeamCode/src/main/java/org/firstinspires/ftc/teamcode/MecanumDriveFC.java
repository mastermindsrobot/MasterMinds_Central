package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Mecanum Drive Field-Centric", group = "TeleOp")
public class MecanumDriveFC extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    private DcMotor intake;

    private GoBildaPinpointDriver pinpoint;

    // false = slow mode, true = fast mode
    private boolean speedToggle = false;

    // Used to detect a new left-bumper press
    private boolean leftBumperWasPressed = false;

    // =========================
    // INTAKE STATE
    // =========================
    //
    //  1  = intake in
    // -1  = intake out
    //  0  = stopped

    private int intakeState = 0;

    // Used to detect individual button presses
    private boolean yWasPressed = false;
    private boolean aWasPressed = false;

    @Override
    public void runOpMode() {

        // =========================
        // MAP MOTORS
        // =========================

        leftFront = hardwareMap.get(DcMotor.class, "front-left");
        rightFront = hardwareMap.get(DcMotor.class, "front-right");
        leftBack = hardwareMap.get(DcMotor.class, "back-left");
        rightBack = hardwareMap.get(DcMotor.class, "back-right");

        // Intake motor
        intake = hardwareMap.get(DcMotor.class, "intake");

        // =========================
        // MAP PINPOINT
        // =========================

        pinpoint = hardwareMap.get(
                GoBildaPinpointDriver.class,
                "pinpoint"
        );

        // =========================
        // MOTOR DIRECTIONS
        // =========================

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        // Intake direction
        // Change FORWARD to REVERSE if necessary
        // after testing the physical direction.

        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        // =========================
        // MOTOR MODES
        // =========================

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // =========================
        // RESET PINPOINT
        // =========================
        //
        // Robot must be stationary here.
        // This establishes the current heading as 0 degrees.

        pinpoint.resetPosAndIMU();

        telemetry.addLine("Mecanum Drive Ready");
        telemetry.addLine("Pinpoint Initialized");
        telemetry.addLine("Intake Ready");
        telemetry.addLine("Keep robot stationary until start.");
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
            // UPDATE PINPOINT
            // =========================

            pinpoint.update();

            double heading = pinpoint.getHeading(
                    AngleUnit.RADIANS
            );

            // =========================
            // SPEED TOGGLE
            // =========================

            boolean leftBumperNow = gamepad1.left_bumper;

            // Detect only the initial press
            if (leftBumperNow && !leftBumperWasPressed) {
                speedToggle = !speedToggle;
            }

            leftBumperWasPressed = leftBumperNow;

            // Slow = 50%
            // Fast = 80%
            double speedMultiplier = speedToggle ? 0.8 : 0.5;

            // =========================
            // GAMEPAD DRIVE INPUT
            // =========================

            // Forward/backward
            double y = -gamepad1.right_stick_y;

            // Strafing
            double x = gamepad1.right_stick_x;

            // Rotation
            double rx = gamepad1.left_stick_x;

            // =========================
            // FIELD-CENTRIC TRANSFORM
            // =========================

            double rotX = x * Math.cos(-heading)
                    - y * Math.sin(-heading);

            double rotY = x * Math.sin(-heading)
                    + y * Math.cos(-heading);

            // =========================
            // STRAFE COMPENSATION
            // =========================

            rotX *= 1.1;

            // =========================
            // MECANUM CALCULATION
            // =========================

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

            // =========================
            // APPLY DRIVE POWER
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
            // INTAKE TOGGLE CONTROL
            // =========================

            boolean yNow = gamepad1.y;
            boolean aNow = gamepad1.a;

            // Y was just pressed
            if (yNow && !yWasPressed) {

                if (intakeState == 1) {
                    // Already intaking -> stop
                    intakeState = 0;
                } else {
                    // Start / switch to intake IN
                    intakeState = 1;
                }
            }

            // A was just pressed
            if (aNow && !aWasPressed) {

                if (intakeState == -1) {
                    // Already intaking OUT -> stop
                    intakeState = 0;
                } else {
                    // Start / switch to intake OUT
                    intakeState = -1;
                }
            }

            // Remember current button states
            yWasPressed = yNow;
            aWasPressed = aNow;

            // =========================
            // APPLY INTAKE POWER
            // =========================

            if (intakeState == 1) {

                // Intake IN
                intake.setPower(1.0);

            } else if (intakeState == -1) {

                // Intake OUT
                intake.setPower(-1.0);

            } else {

                // Intake STOPPED
                intake.setPower(0.0);
            }

            // =========================
            // TELEMETRY
            // =========================

            telemetry.addData(
                    "Heading",
                    "%.1f degrees",
                    Math.toDegrees(heading)
            );

            telemetry.addData(
                    "Speed Mode",
                    speedToggle ? "FAST" : "SLOW"
            );

            telemetry.addData(
                    "Field X",
                    "%.2f",
                    rotX
            );

            telemetry.addData(
                    "Field Y",
                    "%.2f",
                    rotY
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

            // Intake telemetry
            if (intakeState == 1) {

                telemetry.addData("Intake", "IN");

            } else if (intakeState == -1) {

                telemetry.addData("Intake", "OUT");

            } else {

                telemetry.addData("Intake", "STOPPED");
            }

            telemetry.update();
        }
    }
}