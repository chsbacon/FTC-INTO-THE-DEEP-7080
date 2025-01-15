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
import org.firstinspires.ftc.teamcode.drive.modules.Arm.ArmController;
import org.firstinspires.ftc.teamcode.drive.modules.Arm.*;

public class Robot2024 {
    LinearOpMode opMode;
    public HardwareMap hardwareMap;
    public DriveController driveController = null;
    public DcMotorEx linearExtenderMotorL;
    public DcMotorEx linearExtenderMotorR;
    public DcMotorEx forearmEncoder;
    public DcMotorEx armRotationMotorR;
    public DcMotorEx armRotationMotorL; // TODO: 1/14/25 Add arm rotation motor(L&R) and wrist servo to hardware map
    public Servo wristServo;
    public Servo clawServo;
    public ArmController armController = null;
    public boolean doAuto = false;
    MecanumDrive2024 drive;
    Telemetry telemetry;
    WebcamName webcam;
    public boolean demoMode = false;

    public Robot2024(LinearOpMode opMode, MecanumDrive2024 drive, boolean doDriveController, boolean doArmController, boolean doAuto){
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


            armRotationMotorR = this.hardwareMap.get(DcMotorEx.class, "armRotationR");
            armRotationMotorL = this.hardwareMap.get(DcMotorEx.class, "armRotationL");
            // TODO: Set the direction of the motors they will most likely need adjustment as the motors are mounted in different directions

            wristServo = this.hardwareMap.get(Servo.class, "wrist"); //HW map declaration

            forearmEncoder = this.hardwareMap.get(DcMotorEx.class,"forearmEncoder");
            forearmEncoder.setDirection(DcMotorSimple.Direction.FORWARD);
            clawServo = this.hardwareMap.get(Servo.class, "claw"); //HW map declaration
        }
        if (doAuto) {
            this.doAuto = true;
        }
    }
    public Robot2024(LinearOpMode opMode, MecanumDrive2024 drive){
        this(opMode, drive, false, false, false);
    }
    public void onOpmodeInit(){
        //drive.imu.resetYaw();
        if (driveController != null) {
            this.telemetry.log().add("initting drive...");
            driveController.onOpmodeInit(this, this.drive, this.telemetry);
        }
        if (armController != null){
            this.telemetry.log().add("initting arm...");
            armController.onOpmodeInit(this, this.telemetry);
        }
        if (this.doAuto) { //close claw on start of auto
            this.armController.setCurrentState(new Neutral());
            this.telemetry.log().add("inniting auto...");
        }
        this.telemetry.update();
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
