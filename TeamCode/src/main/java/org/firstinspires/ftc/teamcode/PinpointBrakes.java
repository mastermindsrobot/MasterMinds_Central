package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Mecanum Drive with Pinpoint Brake", group = "TeleOp")
public class PinpointBrakes extends LinearOpMode {

    private DcMotor leftFront, rightFront, leftBack, rightBack;
    private GoBildaPinpointDriver pinpoint;

    // =========================================================
    // NORMAL DRIVE
    // =========================================================

    private boolean speedToggle = false;
    private boolean leftBumperWasPressed = false;

    // =========================================================
    // BRAKE
    // =========================================================

    private boolean brakeActive = false;
    private boolean aWasPressed = false;

    // Target pose captured when brake is activated
    private double brakeTargetX = 0.0;
    private double brakeTargetY = 0.0;
    private double brakeTargetHeading = 0.0;

    // =========================================================
    // BRAKE TUNING
    // =========================================================

    /*
     * Position proportional gain.
     *
     * Larger = stronger correction when pushed away.
     * Smaller = softer correction.
     *
     * Start here and tune carefully.
     */
    private static final double BRAKE_KP_XY = 0.045;

    /*
     * Velocity damping.
     *
     * This opposes motion and helps prevent oscillation.
     */
    private static final double BRAKE_KD_XY = 0.020;

    /*
     * Heading proportional gain.
     */
    private static final double BRAKE_KP_HEADING = 0.018;

    /*
     * Heading velocity damping.
     */
    private static final double BRAKE_KD_HEADING = 0.010;

    /*
     * Position deadzone.
     *
     * If the robot is within this distance of the target,
     * no positional correction is applied.
     */
    private static final double POSITION_DEADZONE = 0.35; // inches

    /*
     * Heading deadzone.
     */
    private static final double HEADING_DEADZONE = 1.5; // degrees

    /*
     * Maximum translational brake power.
     *
     * Start relatively low.
     */
    private static final double MAX_BRAKE_TRANSLATION = 0.65;

    /*
     * Maximum rotational brake power.
     */
    private static final double MAX_BRAKE_ROTATION = 0.45;

    /*
     * Prevent extremely tiny motor commands from
     * causing motor chatter.
     */
    private static final double MIN_BRAKE_POWER = 0.05;

    @Override
    public void runOpMode() {

        // =========================================================
        // MAP MOTORS
        // =========================================================

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

        // =========================================================
        // MAP PINPOINT
        // =========================================================

        pinpoint = hardwareMap.get(
                GoBildaPinpointDriver.class,
                "pinpoint"
        );

        // =========================================================
        // MOTOR DIRECTIONS
        // =========================================================

        /*
         * These are your existing working directions.
         * DO NOT change them.
         */

        leftFront.setDirection(
                DcMotorSimple.Direction.FORWARD
        );

        leftBack.setDirection(
                DcMotorSimple.Direction.FORWARD
        );

        rightFront.setDirection(
                DcMotorSimple.Direction.REVERSE
        );

        rightBack.setDirection(
                DcMotorSimple.Direction.REVERSE
        );

        // =========================================================
        // MOTOR MODES
        // =========================================================

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

        // =========================================================
        // PINPOINT CONFIGURATION
        // =========================================================

        /*
         * Your working Pinpoint configuration.
         *
         * X encoder = parallel pod
         * Y encoder = perpendicular pod
         */

        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );

        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        // Reset Pinpoint position and heading
        pinpoint.resetPosAndIMU();

        telemetry.addLine("READY");
        telemetry.addLine("");
        telemetry.addLine("A = Toggle Pinpoint Brake");
        telemetry.addLine("Left Bumper = Toggle Speed");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        // =========================================================
        // MAIN LOOP
        // =========================================================

        while (opModeIsActive()) {

            // Update Pinpoint first
            pinpoint.update();

            // =====================================================
            // READ PINPOINT
            // =====================================================

            double posX = pinpoint.getPosX(
                    DistanceUnit.INCH
            );

            double posY = pinpoint.getPosY(
                    DistanceUnit.INCH
            );

            double heading = pinpoint.getHeading(
                    AngleUnit.RADIANS
            );

            double headingDegrees = Math.toDegrees(
                    heading
            );

            double velocityX = pinpoint.getVelX(
                    DistanceUnit.INCH
            );

            double velocityY = pinpoint.getVelY(
                    DistanceUnit.INCH
            );

            // =====================================================
            // A BUTTON - TOGGLE BRAKE
            // =====================================================

            boolean aNow = gamepad1.a;

            if (aNow && !aWasPressed) {

                if (!brakeActive) {

                    // ---------------------------------------------
                    // ACTIVATE BRAKE
                    // ---------------------------------------------

                    /*
                     * Capture the robot's CURRENT pose.
                     *
                     * This becomes the position the robot
                     * is going to hold.
                     */

                    brakeTargetX = posX;
                    brakeTargetY = posY;
                    brakeTargetHeading = headingDegrees;

                    brakeActive = true;

                } else {

                    // ---------------------------------------------
                    // RELEASE BRAKE
                    // ---------------------------------------------

                    brakeActive = false;

                    // Stop any previous correction
                    leftFront.setPower(0);
                    rightFront.setPower(0);
                    leftBack.setPower(0);
                    rightBack.setPower(0);
                }
            }

            aWasPressed = aNow;

            // =====================================================
            // BRAKE MODE
            // =====================================================

            if (brakeActive) {

                // -------------------------------------------------
                // POSITION ERROR
                // -------------------------------------------------

                double errorX =
                        brakeTargetX - posX;

                double errorY =
                        brakeTargetY - posY;

                // -------------------------------------------------
                // HEADING ERROR
                // -------------------------------------------------

                double errorHeading =
                        brakeTargetHeading - headingDegrees;

                /*
                 * Normalize heading error to -180 to +180.
                 *
                 * This prevents a target of 179 degrees and
                 * current heading of -179 degrees from producing
                 * a 358 degree correction.
                 */

                while (errorHeading > 180.0) {
                    errorHeading -= 360.0;
                }

                while (errorHeading < -180.0) {
                    errorHeading += 360.0;
                }

                // -------------------------------------------------
                // POSITION DEADZONE
                // -------------------------------------------------

                if (Math.abs(errorX) < POSITION_DEADZONE) {
                    errorX = 0.0;
                }

                if (Math.abs(errorY) < POSITION_DEADZONE) {
                    errorY = 0.0;
                }

                // -------------------------------------------------
                // HEADING DEADZONE
                // -------------------------------------------------

                if (Math.abs(errorHeading) < HEADING_DEADZONE) {
                    errorHeading = 0.0;
                }

                // -------------------------------------------------
                // PD TRANSLATIONAL CONTROL
                // -------------------------------------------------

                /*
                 * P component:
                 *
                 * More displacement = more correction.
                 */

                double correctionX =
                        errorX * BRAKE_KP_XY;

                double correctionY =
                        errorY * BRAKE_KP_XY;

                /*
                 * D component:
                 *
                 * If the robot is already moving, oppose that
                 * movement. This is what helps prevent oscillation.
                 */

                correctionX -=
                        velocityX * BRAKE_KD_XY;

                correctionY -=
                        velocityY * BRAKE_KD_XY;

                // -------------------------------------------------
                // CONVERT FIELD ERROR TO ROBOT ERROR
                // -------------------------------------------------

                /*
                 * Pinpoint gives us the error in field coordinates.
                 *
                 * The drivetrain needs robot-relative commands.
                 */

                double robotCorrectionX =
                        correctionX * Math.cos(heading)
                                - correctionY * Math.sin(heading);

                double robotCorrectionY =
                        correctionX * Math.sin(heading)
                                + correctionY * Math.cos(heading);

                // Mecanum strafing compensation
                robotCorrectionX *= 1.1;

                // -------------------------------------------------
                // ROTATIONAL PD CONTROL
                // -------------------------------------------------

                /*
                 * Pinpoint's heading velocity is not available in
                 * your current driver API, so we use the measured
                 * translational velocity only for damping here.
                 *
                 * Start with proportional heading control.
                 */

                double rotationCorrection =
                        errorHeading * BRAKE_KP_HEADING;

                // -------------------------------------------------
                // LIMIT OUTPUTS
                // -------------------------------------------------

                robotCorrectionX = clamp(
                        robotCorrectionX,
                        -MAX_BRAKE_TRANSLATION,
                        MAX_BRAKE_TRANSLATION
                );

                robotCorrectionY = clamp(
                        robotCorrectionY,
                        -MAX_BRAKE_TRANSLATION,
                        MAX_BRAKE_TRANSLATION
                );

                rotationCorrection = clamp(
                        rotationCorrection,
                        -MAX_BRAKE_ROTATION,
                        MAX_BRAKE_ROTATION
                );

                // -------------------------------------------------
                // MECANUM CALCULATION
                // -------------------------------------------------

                double denominator = Math.max(
                        Math.abs(robotCorrectionY)
                                + Math.abs(robotCorrectionX)
                                + Math.abs(rotationCorrection),
                        1.0
                );

                double frontLeftPower =
                        (robotCorrectionY
                                + robotCorrectionX
                                + rotationCorrection)
                                / denominator;

                double backLeftPower =
                        (robotCorrectionY
                                - robotCorrectionX
                                + rotationCorrection)
                                / denominator;

                double frontRightPower =
                        (robotCorrectionY
                                - robotCorrectionX
                                - rotationCorrection)
                                / denominator;

                double backRightPower =
                        (robotCorrectionY
                                + robotCorrectionX
                                - rotationCorrection)
                                / denominator;

                // -------------------------------------------------
                // APPLY BRAKE POWER
                // -------------------------------------------------

                leftFront.setPower(frontLeftPower);
                leftBack.setPower(backLeftPower);

                rightFront.setPower(frontRightPower);
                rightBack.setPower(backRightPower);

                // -------------------------------------------------
                // BRAKE TELEMETRY
                // -------------------------------------------------

                double distanceError = Math.sqrt(
                        errorX * errorX
                                + errorY * errorY
                );

                telemetry.addLine(
                        "========== BRAKE ACTIVE =========="
                );

                telemetry.addData(
                        "Target X",
                        "%.2f in",
                        brakeTargetX
                );

                telemetry.addData(
                        "Target Y",
                        "%.2f in",
                        brakeTargetY
                );

                telemetry.addData(
                        "Current X",
                        "%.2f in",
                        posX
                );

                telemetry.addData(
                        "Current Y",
                        "%.2f in",
                        posY
                );

                telemetry.addData(
                        "Distance Error",
                        "%.2f in",
                        distanceError
                );

                telemetry.addData(
                        "Heading Error",
                        "%.2f deg",
                        errorHeading
                );

                telemetry.addData(
                        "Brake FL",
                        "%.2f",
                        frontLeftPower
                );

                telemetry.addData(
                        "Brake BL",
                        "%.2f",
                        backLeftPower
                );

                telemetry.addData(
                        "Brake FR",
                        "%.2f",
                        frontRightPower
                );

                telemetry.addData(
                        "Brake BR",
                        "%.2f",
                        backRightPower
                );

                telemetry.update();

                /*
                 * IMPORTANT:
                 *
                 * Do NOT execute the normal joystick driving code
                 * below while brake mode is active.
                 */

                continue;
            }

            // =====================================================
            // NORMAL SPEED TOGGLE
            // =====================================================

            boolean leftBumperNow =
                    gamepad1.left_bumper;

            if (leftBumperNow && !leftBumperWasPressed) {
                speedToggle = !speedToggle;
            }

            leftBumperWasPressed =
                    leftBumperNow;

            double speedMultiplier =
                    speedToggle ? 1.0 : 0.5;

            // =====================================================
            // NORMAL GAMEPAD INPUT
            // =====================================================

            double y =
                    gamepad1.right_stick_y;

            double x =
                    -gamepad1.right_stick_x;

            double rx =
                    -gamepad1.left_stick_x;

            // =====================================================
            // FIELD-CENTRIC TRANSFORM
            // =====================================================

            double rotX =
                    x * Math.cos(heading)
                            - y * Math.sin(heading);

            double rotY =
                    x * Math.sin(heading)
                            + y * Math.cos(heading);

            rotX *= 1.1;

            // =====================================================
            // MECANUM CALCULATION
            // =====================================================

            double denominator = Math.max(
                    Math.abs(rotY)
                            + Math.abs(rotX)
                            + Math.abs(rx),
                    1.0
            );

            double frontLeftPower =
                    (rotY + rotX + rx)
                            / denominator;

            double backLeftPower =
                    (rotY - rotX + rx)
                            / denominator;

            double frontRightPower =
                    (rotY - rotX - rx)
                            / denominator;

            double backRightPower =
                    (rotY + rotX - rx)
                            / denominator;

            // =====================================================
            // APPLY MOTOR POWER
            // =====================================================

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

            // =====================================================
            // TELEMETRY
            // =====================================================

            telemetry.addLine(
                    "========== NORMAL DRIVE =========="
            );

            telemetry.addData(
                    "X",
                    "%.2f in",
                    posX
            );

            telemetry.addData(
                    "Y",
                    "%.2f in",
                    posY
            );

            telemetry.addData(
                    "Heading",
                    "%.1f deg",
                    headingDegrees
            );

            telemetry.addData(
                    "Speed Mode",
                    speedToggle ? "Fast" : "Slow"
            );

            telemetry.addData(
                    "Brake",
                    "OFF - Press A"
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

    // =========================================================
    // CLAMP FUNCTION
    // =========================================================

    private double clamp(
            double value,
            double min,
            double max
    ) {
        return Math.max(
                min,
                Math.min(max, value)
        );
    }
}