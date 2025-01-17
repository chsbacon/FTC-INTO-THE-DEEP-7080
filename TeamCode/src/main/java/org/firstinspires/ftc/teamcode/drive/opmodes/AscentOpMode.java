package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.hardware.MecanumDrive2024;
import org.firstinspires.ftc.teamcode.drive.modules.Robot2024;

@TeleOp(name="Ascent Teleop")
public class AscentOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive2024 drive = new MecanumDrive2024(hardwareMap);
        drive.imu.resetYaw();
        Robot2024 robot = new Robot2024(this, drive, true, true,false);
        robot.onOpmodeInit();
        waitForStart();
        /*
        robot.armController.ascentForearm();
        robot.armController.homing();
         */
        while (opModeIsActive()){
            robot.doLoop(gamepad1, gamepad2);
            idle();
        }
    }
}
