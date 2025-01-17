package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.hardware.MecanumDrive2024;
import org.firstinspires.ftc.teamcode.drive.modules.Robot2024;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class LeftBasketPark extends LinearOpMode {
    Robot2024 robot;
    public int depositTime = 2; //In seconds, amount of time it takes for the claw to retract
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive2024 drive = new MecanumDrive2024(hardwareMap);
        robot = new Robot2024(this, drive, true, true,true);
        robot.onOpmodeInit();
        drive.imu.resetYaw();
        Pose2d start = new Pose2d(-12, -60, Math.toRadians(90.00));
        drive.setPoseEstimate(start);

        TrajectorySequence myTrajectory = drive.trajectorySequenceBuilder(start)
                .addDisplacementMarker(() -> { //This can run while the path runs, hence why we don't need a wait call
//              TODO: Needs to implement state machine

                })
                .splineTo(new Vector2d(-53, -53), Math.toRadians(224.17))
                .addDisplacementMarker(() -> { //Must not be interrupted by trajectory, requires a wait
//                    TODO: Needs to implement state machine
                })
                .waitSeconds(depositTime)
                //.addDisplacementMarker(() -> robot.armController.goToLinear(0, 1)) //Lower arm back to 0
                .back(12)
                .setReversed(true)
                .splineTo(new Vector2d(-26, 0), Math.toRadians(0))
                .build();

        waitForStart();
        if (!isStopRequested())
            drive.followTrajectorySequence(myTrajectory);
    }
}