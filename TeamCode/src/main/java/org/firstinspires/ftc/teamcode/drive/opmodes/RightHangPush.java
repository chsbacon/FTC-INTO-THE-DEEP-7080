package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.hardware.MecanumDrive2024;
import org.firstinspires.ftc.teamcode.drive.modules.Robot2024;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import kotlin.Deprecated;

@Autonomous (name="Right Hang Push", preselectTeleOp="Standard TeleOp")
public class RightHangPush extends LinearOpMode {
    Robot2024 robot;
    public double subPlacementTime = 5; //In seconds. This is way too high, but better to wait more. Testing can shorten this it's just a guess
    boolean running = false;

    public void runOpMode() throws InterruptedException {
        MecanumDrive2024 drive = new MecanumDrive2024(hardwareMap);
        robot = new Robot2024(this, drive, true, true,true);
        robot.onOpmodeInit();
        drive.imu.resetYaw();
        Pose2d start = new Pose2d(12, -60, Math.toRadians(90.00));
        drive.setPoseEstimate(start);

        TrajectorySequence myTrajectory = drive.trajectorySequenceBuilder(start)
                /*
                .waitSeconds(6)
                //running at the same time (both calls are immediate)
                .UNSTABLE_addDisplacementMarkerOffset(0,() -> { //This can run while the path runs, hence why we don't need a wait call
                    robot.armController.goToLinear(1200, 0.5); //Raise arm
                    robot.armController.goToForearmAuto(robot.armController.FOREARM_VERT, 1);
                })
                .splineTo(new Vector2d(0, -30), Math.toRadians(90.00))
                //running at the same time (lower arm for 3 seconds, open claw for 2)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> { //THIS MUST RUN UNINTERRUPTED BY TRAJECTORY. We use a wait here to ensure the robot doesn't start moving while it's still placing
                    robot.armController.goToLinear(100, 0.5); //Lower arm to hook (slowly)
                })
                .UNSTABLE_addTemporalMarkerOffset(3,() -> { //THIS MUST RUN UNINTERRUPTED BY TRAJECTORY. We use a wait here to ensure the robot doesn't start moving while it's still placing
                    robot.armController.doClawControl(true); //Open claw
                })
                .waitSeconds(subPlacementTime)
                //running at the same (return forearm 0 seconds ino wait)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> { //THIS MUST RUN UNINTERRUPTED BY TRAJECTORY. We use a wait here to ensure the robot doesn't start moving while it's still placing
                    robot.armController.goToForearmAuto(150, 1); //Return forearm to back
                })
                .UNSTABLE_addTemporalMarkerOffset(0,() -> { //THIS MUST RUN UNINTERRUPTED BY TRAJECTORY. We use a wait here to ensure the robot doesn't start moving while it's still placing
                    robot.armController.goToLinear(400, 0.5); // raise arm to disengage with sub
                })
                .waitSeconds(subPlacementTime)
                //running at the same time
                .UNSTABLE_addTemporalMarkerOffset(0,() -> robot.armController.goToLinear(0, 1)) //Lower arm back to 0
                .setReversed(true)
                .lineTo(new Vector2d(0, -37.40))
                .setReversed(false)
                .lineTo(new Vector2d(55,-55))
                .turn(Math.toRadians(90))

                 */
                .build();
        waitForStart();
        if (!isStopRequested())
            drive.followTrajectorySequence(myTrajectory);
    }
}