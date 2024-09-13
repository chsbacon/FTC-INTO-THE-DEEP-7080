package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.hardware.MecanumDrive2024;
import org.firstinspires.ftc.teamcode.drive.modules.Robot2024;

@TeleOp
public class IntegratedOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive2024 drive = new MecanumDrive2024(hardwareMap);
        drive.imu.resetYaw();
        Robot2024 robot = new Robot2024(this, drive, true);
        robot.onOpmodeInit();
        waitForStart();
        while (opModeIsActive()){
            robot.doLoop(gamepad1, gamepad2);
            idle();
        }
    }
}
