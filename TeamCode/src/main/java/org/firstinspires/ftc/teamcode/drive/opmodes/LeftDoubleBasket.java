package org.firstinspires.ftc.teamcode.drive.opmodes;

import static org.firstinspires.ftc.teamcode.drive.opmodes.LeftBasketPark.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.hardware.MecanumDrive2024;
import org.firstinspires.ftc.teamcode.drive.modules.Arm.ArmController;
import org.firstinspires.ftc.teamcode.drive.modules.Robot2024;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="Left Double Basket", preselectTeleOp="Ascent TeleOp")
public class LeftDoubleBasket extends LinearOpMode {
    Robot2024 robot;
    public int depositTime = 2; //In seconds, amount of time it takes for the claw to retract
    boolean running = false;
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive2024 drive = new MecanumDrive2024(hardwareMap);
        robot = new Robot2024(this, drive, true, true,true);
        robot.onOpmodeInit();
        drive.imu.resetYaw();
        Pose2d start = new Pose2d(-12, -60, Math.toRadians(90.00));
        drive.setPoseEstimate(start);

        TrajectorySequence myTrajectory = drive.trajectorySequenceBuilder(start)
                /*
                .UNSTABLE_addDisplacementMarkerOffset(0,() -> { //This can run while the path runs, hence why we don't need a wait call
                    robot.armController.goToLinear(5200, 0.75); //Raise arm to top
                    robot.armController.goToForearmAuto(robot.armController.FOREARM_VERT, 1); //Vertical arm
                })
                .splineTo(new Vector2d(-64, -64), Math.toRadians(225))
                .UNSTABLE_addTemporalMarkerOffset(1,() -> { //THIS MUST RUN UNINTERRUPTED BY TRAJECTORY. We use a wait here to ensure the robot doesn't start moving while it's still placing
                    robot.armController.doClawControl(true); //Open claw
                })
                .waitSeconds(2)
                .lineTo(new Vector2d(-45,-50))
                .UNSTABLE_addTemporalMarkerOffset(0,() -> { //Lower while splining
                    robot.armController.goToLinear(0, 0.75); //Lower arm to 0
                })
                .turn(Math.toRadians(-135))
                //Pick up specimen and score
                .lineTo(new Vector2d(-54,-55))
                .UNSTABLE_addTemporalMarkerOffset(0,() -> { //This can run while the path runs, hence why we don't need a wait call
                    robot.armController.goToForearmAuto(robot.armController.FOREARM_HORIZ, 1); //Vertical arm
                })
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> { //This can run while the path runs, hence why we don't need a wait call
                    robot.armController.doClawControl(false); //Close claw
                })
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> { //This can run while the path runs, hence why we don't need a wait call
                    robot.armController.goToLinear(5200, 0.75); //Raise arm to top
                    robot.armController.goToForearmAuto(robot.armController.FOREARM_VERT, 1); //Vertical arm
                })
                .waitSeconds(3)
                .turn(Math.toRadians(135))
                .lineTo(new Vector2d(-65,-65))
                .UNSTABLE_addTemporalMarkerOffset(1,() -> { //THIS MUST RUN UNINTERRUPTED BY TRAJECTORY. We use a wait here to ensure the robot doesn't start moving while it's still placing
                    robot.armController.doClawControl(true); //Open claw
                })
                .waitSeconds(2)
                .lineTo(new Vector2d(-45,-45))
                .UNSTABLE_addTemporalMarkerOffset(0,() -> { //Lower while splining
                    robot.armController.goToLinear(0, 0.75); //Lower arm to 0
                })
                .UNSTABLE_addTemporalMarkerOffset(0,() -> { //Lower while splining
                    robot.armController.goToForearmAuto(150, 1); //Lower arm to 0
                })
                .splineToSplineHeading(new Pose2d(-50, -12), Math.toRadians(90))
                .lineTo(new Vector2d(-32,-12))
                .turn(Math.toRadians(90))

                 */
                .build();
        waitForStart();
        if (!isStopRequested())
            drive.followTrajectorySequence(myTrajectory);
    }
}