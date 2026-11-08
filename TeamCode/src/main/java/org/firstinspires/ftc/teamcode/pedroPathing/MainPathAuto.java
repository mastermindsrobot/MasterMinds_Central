package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Main Path", group = "Pedro Pathing")
public class MainPathAuto extends OpMode {

    private Follower follower;
    private Paths paths;

    // Starting pose from Pedro Pathing Visualizer
    private final Pose startPose =
            new Pose(
                    56.000,
                    8.000,
                    Math.toRadians(90)
            );

    @Override
    public void init() {

        // -----------------------------------------
        // CREATE PEDRO FOLLOWER
        // -----------------------------------------

        follower = Constants.createFollower(hardwareMap);

        // Tell Pedro the robot's starting position
        follower.setStartingPose(startPose);

        follower.setMaxPower(0.55);

        // Build the complete path
        paths = new Paths(follower);

        telemetry.addLine("Pedro Pathing Ready");
        telemetry.addLine("SLOW MODE");
        telemetry.addLine("");

        telemetry.addData(
                "Start X",
                "%.2f",
                startPose.getX()
        );

        telemetry.addData(
                "Start Y",
                "%.2f",
                startPose.getY()
        );

        telemetry.addData(
                "Start Heading",
                "%.1f°",
                Math.toDegrees(startPose.getHeading())
        );

        telemetry.addLine("");
        telemetry.addLine("Press PLAY to run");

        telemetry.update();
    }

    @Override
    public void start() {

        // -----------------------------------------
        // START PATH
        // -----------------------------------------

        follower.followPath(paths.MainChain);
    }

    @Override
    public void loop() {

        // -----------------------------------------
        // UPDATE PEDRO
        // -----------------------------------------

        follower.update();

        // -----------------------------------------
        // TELEMETRY
        // -----------------------------------------

        Pose currentPose = follower.getPose();

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
                "Busy",
                follower.isBusy()
        );

        telemetry.addData(
                "Status",
                follower.isBusy()
                        ? "FOLLOWING PATH"
                        : "PATH COMPLETE"
        );

        telemetry.update();
    }

    // =========================================================
    // GENERATED PATH
    // =========================================================

    public static class Paths {
        public PathChain MainChain;

        public Paths(Follower follower) {
            MainChain = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(56.000, 8.000),
                                    new Pose(96.702, 9.905),
                                    new Pose(97.586, 34.155)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                    .addPath(
                            new BezierLine(
                                    new Pose(97.586, 34.155),
                                    new Pose(127.002, 34.667)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .addPath(
                            new BezierCurve(
                                    new Pose(127.002, 34.667),
                                    new Pose(91.824, 40.141),
                                    new Pose(95.534, 59.423)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .addPath(
                            new BezierLine(
                                    new Pose(95.534, 59.423),
                                    new Pose(128.082, 58.796)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .addPath(
                            new BezierCurve(
                                    new Pose(128.082, 58.796),
                                    new Pose(98.904, 66.259),
                                    new Pose(94.659, 81.972)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .addPath(
                            new BezierLine(
                                    new Pose(94.659, 81.972),
                                    new Pose(128.912, 81.745)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .addPath(
                            new BezierLine(
                                    new Pose(128.912, 81.745),
                                    new Pose(83.092, 82.248)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .addPath(
                            new BezierCurve(
                                    new Pose(83.092, 82.248),
                                    new Pose(8.902, 63.245),
                                    new Pose(71.188, 22.967)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                    .build();
        }
    }
}