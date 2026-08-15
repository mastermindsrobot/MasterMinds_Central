package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Constants {

    /*
     * =========================
     * DRIVETRAIN CONFIGURATION
     * =========================
     */

    public static MecanumConstants driveConstants =
            new MecanumConstants()
                    .leftFrontMotorName("front-left")
                    .leftRearMotorName("back-left")
                    .rightFrontMotorName("front-right")
                    .rightRearMotorName("back-right")

                    .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
                    .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
                    .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                    .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);
    /*
     * =========================
     * PINPOINT CONFIGURATION
     * =========================
     */

    public static PinpointConstants localizerConstants =
            new PinpointConstants()
                    .forwardPodY(0.5)
                    .strafePodX(2.8)
                    .hardwareMapName("pinpoint")
                    .forwardEncoderDirection(
                            GoBildaPinpointDriver.EncoderDirection.FORWARD
                    )
                    .strafeEncoderDirection(
                            GoBildaPinpointDriver.EncoderDirection.FORWARD
                    );


    /*
     * =========================
     * FOLLOWER CONFIGURATION
     * =========================
     */

    public static FollowerConstants followerConstants =
            new FollowerConstants();


    /*
     * =========================
     * PATH CONSTRAINTS
     * =========================
     */

    public static PathConstraints pathConstraints =
            new PathConstraints(0.99, 100, 1, 1);


    /*
     * =========================
     * CREATE FOLLOWER
     * =========================
     */

    public static Follower createFollower(HardwareMap hardwareMap) {

        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}