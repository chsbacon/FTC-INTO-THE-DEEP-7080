package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.hardware.MecanumDrive2024;
import org.firstinspires.ftc.teamcode.drive.modules.Robot2024;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class AutonomousOpMode extends LinearOpMode {
    Robot2024 robot;
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive2024 drive = new MecanumDrive2024(hardwareMap);
        robot = new Robot2024(this, drive, true, true);
        robot.onOpmodeInit();
        drive.imu.resetYaw();
        Pose2d start = new Pose2d(11, -60, Math.toRadians(90.00));
        drive.setPoseEstimate(start);

        TrajectorySequence myTrajectory = drive.trajectorySequenceBuilder(start)
                .splineTo(new Vector2d(0, -33), Math.toRadians(90.00))
                .setReversed(true)
                .lineTo(new Vector2d(0, -37.40))
                .setReversed(false)
                .turn(Math.toRadians(-90))
                .splineTo(new Vector2d(58.49, -58), Math.toRadians(0))
                .build();

        waitForStart();
        if (!isStopRequested())
            drive.followTrajectorySequence(myTrajectory);
    }
}