package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.hardware.MecanumDrive2024;
import org.firstinspires.ftc.teamcode.drive.modules.Robot2024;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class AutonomousRightOpMode extends LinearOpMode {
    Robot2024 robot;
    public double subPlacementTime = 10; //In seconds. This is way too high, but better to wait more. Testing can shorten this it's just a guess

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive2024 drive = new MecanumDrive2024(hardwareMap);
        robot = new Robot2024(this, drive, true, true,true);
        robot.onOpmodeInit();
        drive.imu.resetYaw();
        Pose2d start = new Pose2d(12, -60, Math.toRadians(90.00));
        drive.setPoseEstimate(start);

        TrajectorySequence myTrajectory = drive.trajectorySequenceBuilder(start)
                .addDisplacementMarker(() -> { //This can run while the path runs, hence why we don't need a wait call

                })
                .splineTo(new Vector2d(0, -33), Math.toRadians(90.00))
                .addDisplacementMarker(() -> { //THIS MUST RUN UNINTERRUPTED BY TRAJECTORY. We use a wait here to ensure the robot doesn't start moving while it's still placing
//                   TODO: Needs to implement state machine
                })
                .waitSeconds(subPlacementTime /3)
                .addDisplacementMarker(() -> { //THIS MUST RUN UNINTERRUPTED BY TRAJECTORY. We use a wait here to ensure the robot doesn't start moving while it's still placing

                })
                .waitSeconds(subPlacementTime /3)
                .addDisplacementMarker(() -> { //THIS MUST RUN UNINTERRUPTED BY TRAJECTORY. We use a wait here to ensure the robot doesn't start moving while it's still placing
                    robot.armController.goToForearm(0, 1); //Return forearm to back
                })
                .waitSeconds(subPlacementTime /3)
                .addDisplacementMarker(() -> robot.armController.goToLinear(0, 1)) //Lower arm back to 0
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