package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Pedro Path TeleOp", group = "TeleOp")
public class PathTeleOp extends OpMode {

    private Follower follower;

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    private final Pose startPose =
            new Pose(72, 72, Math.toRadians(0));

    private final Pose targetPose =
            new Pose(112, 42, Math.toRadians(0));

    private boolean lastA = false;

    private boolean pathRunning = false;

    private boolean speedToggle = false;

    private boolean lastLeftBumper = false;

    @Override
    public void init() {

        // -----------------------------------------
        // PEDRO
        // -----------------------------------------

        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(startPose);


        // -----------------------------------------
        // MOTORS
        // -----------------------------------------

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


        // -----------------------------------------
        // MOTOR DIRECTIONS
        // -----------------------------------------

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


        // -----------------------------------------
        // MOTOR MODE
        // -----------------------------------------

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


        telemetry.addLine("Pedro Path TeleOp Ready");
        telemetry.addLine("");
        telemetry.addLine("Left Stick = Drive");
        telemetry.addLine("Right Stick X = Rotate");
        telemetry.addLine("A = Go to target");
        telemetry.addLine("Left Bumper = Toggle speed");
        telemetry.addLine("");
        telemetry.addData("Start X", startPose.getX());
        telemetry.addData("Start Y", startPose.getY());
        telemetry.addData("Target X", targetPose.getX());
        telemetry.addData("Target Y", targetPose.getY());
        telemetry.update();
    }


    @Override
    public void start() {

        follower.startTeleopDrive();
    }


    @Override
    public void loop() {

        // -----------------------------------------
        // UPDATE PEDRO
        // -----------------------------------------

        follower.update();


        // -----------------------------------------
        // SPEED TOGGLE
        // -----------------------------------------

        boolean leftBumperNow = gamepad1.left_bumper;

        if (leftBumperNow && !lastLeftBumper) {
            speedToggle = !speedToggle;
        }

        lastLeftBumper = leftBumperNow;

        double speedMultiplier =
                speedToggle ? 1.0 : 0.5;


        // -----------------------------------------
        // A BUTTON
        // -----------------------------------------

        boolean aNow = gamepad1.a;

        if (aNow && !lastA && !pathRunning) {

            /*
             * IMPORTANT:
             *
             * Get the robot's CURRENT Pedro pose.
             *
             * Therefore, if the robot has been manually
             * driven to (90,60), the path begins at
             * (90,60), NOT at (72,72).
             */

            Pose currentPose = follower.getPose();

            PathChain pathToTarget =
                    follower.pathBuilder()
                            .addPath(
                                    new BezierLine(
                                            currentPose,
                                            targetPose
                                    )
                            )
                            .setLinearHeadingInterpolation(
                                    currentPose.getHeading(),
                                    targetPose.getHeading()
                            )
                            .build();

            follower.followPath(pathToTarget);

            pathRunning = true;
        }

        lastA = aNow;


        // -----------------------------------------
        // PATH FOLLOWING
        // -----------------------------------------

        if (pathRunning) {

            /*
             * Pedro is controlling the motors while
             * following the path.
             */

            if (!follower.isBusy()) {

                pathRunning = false;

                // Stop the motors when path is complete
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
            }

        } else {

            // -----------------------------------------
            // MANUAL MECANUM DRIVE
            // -----------------------------------------

            double y =
                    -gamepad1.left_stick_y;

            double x =
                    gamepad1.left_stick_x;

            double rx =
                    gamepad1.right_stick_x;


            // -----------------------------------------
            // FIELD-CENTRIC DRIVE
            // -----------------------------------------

            double heading =
                    follower.getPose().getHeading();

            double rotX =
                    x * Math.cos(heading)
                            - y * Math.sin(heading);

            double rotY =
                    x * Math.sin(heading)
                            + y * Math.cos(heading);

            // Mecanum strafing compensation
            rotX *= 1.1;


            // -----------------------------------------
            // MECANUM CALCULATION
            // -----------------------------------------

            double denominator =
                    Math.max(
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


            // -----------------------------------------
            // APPLY POWER
            // -----------------------------------------

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
        }


        // -----------------------------------------
        // TELEMETRY
        // -----------------------------------------

        Pose currentPose =
                follower.getPose();

        telemetry.addData(
                "X",
                "%.2f",
                currentPose.getX()
        );

        telemetry.addData(
                "Y",
                "%.2f",
                currentPose.getY()
        );

        telemetry.addData(
                "Heading",
                "%.1f°",
                Math.toDegrees(
                        currentPose.getHeading()
                )
        );

        telemetry.addData(
                "Mode",
                pathRunning
                        ? "FOLLOWING PATH"
                        : "MANUAL"
        );

        telemetry.addData(
                "Speed",
                speedToggle
                        ? "FAST"
                        : "SLOW"
        );

        telemetry.addData(
                "Target",
                "(110, 39)"
        );

        telemetry.update();
    }
}