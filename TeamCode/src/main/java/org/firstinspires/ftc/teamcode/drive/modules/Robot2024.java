package org.firstinspires.ftc.teamcode.drive.modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.hardware.MecanumDrive2024;

public class Robot2024 {
    LinearOpMode opMode;
    public HardwareMap hardwareMap;
    public DriveController driveController = null;
    MecanumDrive2024 drive;
    Telemetry telemetry;
    WebcamName webcam;
    public boolean demoMode = false;

    public Robot2024(LinearOpMode opMode, MecanumDrive2024 drive, boolean doDriveController){
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        this.drive = drive;

        if (doDriveController){
            driveController = new DriveController();
        }
    }
    public Robot2024(LinearOpMode opMode, MecanumDrive2024 drive){
        this(opMode, drive, false);
    }
    public void onOpmodeInit(){
        //drive.imu.resetYaw();
        if (driveController != null) {
            this.telemetry.log().add("initting drive...");
            this.telemetry.update();
            driveController.onOpmodeInit(this, this.drive, this.telemetry);
        }
    }
    public void doLoop(Gamepad gamepad1, Gamepad gamepad2){
        if (driveController != null){
            driveController.doLoop(gamepad1, gamepad2);
        }
    }

}
