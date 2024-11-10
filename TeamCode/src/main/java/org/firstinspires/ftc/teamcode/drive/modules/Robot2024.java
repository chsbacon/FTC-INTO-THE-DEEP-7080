package org.firstinspires.ftc.teamcode.drive.modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.hardware.MecanumDrive2024;
import org.firstinspires.ftc.teamcode.util.Encoder;

public class Robot2024 {
    LinearOpMode opMode;
    public HardwareMap hardwareMap;
    public DriveController driveController = null;
    public DcMotorEx linearExtenderMotorL;
    public DcMotorEx linearExtenderMotorR;
    public DcMotorEx linearRetractor;
    public DcMotorEx forearmEncoder;
    public CRServo forearmServoL;
    public CRServo forearmServoR;
    public Servo clawServo;
    public ArmController armController = null;
    MecanumDrive2024 drive;
    Telemetry telemetry;
    WebcamName webcam;
    public boolean demoMode = false;

    public Robot2024(LinearOpMode opMode, MecanumDrive2024 drive, boolean doDriveController, boolean doArmController){
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        this.drive = drive;

        if (doDriveController){
            driveController = new DriveController();
        }
        if (doArmController) {
            armController = new ArmController();
            linearExtenderMotorL = this.hardwareMap.get(DcMotorEx.class, "linearExtenderL"); //HW map declaration
            linearExtenderMotorL.setDirection(DcMotorSimple.Direction.FORWARD); //Change after tests
            linearExtenderMotorR = this.hardwareMap.get(DcMotorEx.class, "linearExtenderR"); //HW map declaration
            linearExtenderMotorR.setDirection(DcMotorSimple.Direction.REVERSE); //Change after tests
            linearRetractor = this.hardwareMap.get(DcMotorEx.class, "retractor");
            linearRetractor.setDirection(DcMotorSimple.Direction.FORWARD); //Change after tests
            forearmServoL = this.hardwareMap.get(CRServo.class, "forearmServoL"); //HW map declaration
            forearmServoR = this.hardwareMap.get(CRServo.class, "forearmServoR"); //HW map declaration
            forearmServoL.setDirection(DcMotorSimple.Direction.REVERSE);
            forearmServoR.setDirection(DcMotorSimple.Direction.FORWARD);
            forearmEncoder = this.hardwareMap.get(DcMotorEx.class,"forearmEncoder");
            forearmEncoder.setDirection(DcMotorSimple.Direction.FORWARD);
            clawServo = this.hardwareMap.get(Servo.class, "claw"); //HW map declaration
        }
    }
    public Robot2024(LinearOpMode opMode, MecanumDrive2024 drive){
        this(opMode, drive, false, false);
    }
    public void onOpmodeInit(){
        //drive.imu.resetYaw();
        if (driveController != null) {
            this.telemetry.log().add("initting drive...");
            this.telemetry.update();
            driveController.onOpmodeInit(this, this.drive, this.telemetry);
        }
        if (armController != null){
            this.telemetry.log().add("initting arm...");
            this.telemetry.update();
            armController.onOpmodeInit(this, this.telemetry);
        }
    }
    public void doLoop(Gamepad gamepad1, Gamepad gamepad2){
        if (driveController != null){
            driveController.doLoop(gamepad1, gamepad2);
        }
        if (armController != null){
            armController.doLoop(gamepad1, gamepad2);
        }
    }

}
