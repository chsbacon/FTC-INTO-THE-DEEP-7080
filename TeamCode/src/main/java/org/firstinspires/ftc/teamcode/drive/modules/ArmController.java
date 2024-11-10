package org.firstinspires.ftc.teamcode.drive.modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Encoder;

public class ArmController {
    private Robot2024 robot;
    private Telemetry telemetry;
    public int targetAngle = 0;
    public final int LINEAR_MIN = 0;
    public final int LINEAR_MAX = 5580;
    public final int FOREARM_MIN = 0;
    public final int FOREARM_MAX = 2*90;
    public final int Intake = 500;
    public final int LowBasket = 1000;
    public final int HighBasket = LINEAR_MAX;
    public final int LowSub = 1000;
    public final int HighSub = 4000;
    public final int Hang = LowSub+800;
    public final int Margin = 100; //Margin for forearm
    private final int EncoderTPR = 8192;
    private ElapsedTime loopTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private boolean clawOpen = false;
    private final int openClawDeg = 90; // need to adjust these in testing
    private final int closedClawDeg = 120;
    public void onOpmodeInit(Robot2024 robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
        for(DcMotorEx motor: new DcMotorEx[]{robot.linearExtenderMotorL,robot.linearExtenderMotorR,robot.linearRetractor}){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(0);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(1);
        }
        robot.forearmEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.forearmEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.forearmServoL.setPower(0);
        robot.forearmServoR.setPower(0);
    }
    public void doLoop(Gamepad gamepad1, Gamepad gamepad2){

        doManualLinear(gamepad2, gamepad2.start && gamepad2.left_bumper);
        //doClawControl(gamepad2);
        if(gamepad2.a) {
            targetAngle=0;
            goToForearm(targetAngle,1); //Vertical, hopefully
        }
        if(gamepad2.b) {
            targetAngle=90;
            goToForearm(targetAngle,1); //Forwards, hopefully
        }
        if(gamepad2.x) {
            targetAngle=-45;
            goToForearm(targetAngle,1); //Backwards, hopefully
        }
        if(gamepad2.dpad_up) {
            goToLinear(5500, 1.0);
        }
        if(gamepad2.dpad_left) {
            goToLinear(2000, 1.0);
        }
        if(gamepad2.dpad_down) {
            goToLinear(20, 1.0);
        }
        if((angleToTicks(targetAngle)-100 < robot.forearmEncoder.getCurrentPosition())
                && (robot.forearmEncoder.getCurrentPosition() < angleToTicks(targetAngle) + 100)) { //if current position is within +-20 ticks of target
            robot.forearmServoL.setPower(0);
            robot.forearmServoR.setPower(0);
        }
        //doManualForearm(gamepad2);
        if(gamepad2.back){ //Disable motors
            for(DcMotorEx motor: new DcMotorEx[]{robot.linearExtenderMotorL,robot.linearExtenderMotorR,robot.linearRetractor}){
                motor.setPower(0);
            }
        } else {
            robot.linearExtenderMotorL.setPower(1);
            robot.linearExtenderMotorR.setPower(1);
            robot.linearRetractor.setPower(1);
        }

        telemetry.addData("left: linearC: ", robot.linearExtenderMotorL.getCurrentPosition());
        telemetry.addData("right: linearC: ", robot.linearExtenderMotorR.getCurrentPosition());
        telemetry.addData("left: linearT: ", robot.linearExtenderMotorL.getTargetPosition());
        telemetry.addData("right: linearT: ", robot.linearExtenderMotorL.getTargetPosition());
        telemetry.addData("encoder position", robot.forearmEncoder.getCurrentPosition());
        loopTimer.reset();
    }
    public void doClawControl(boolean clawOpen){
        if (clawOpen) {
            robot.clawServo.setPosition(openClawDeg);
        } else {
            robot.clawServo.setPosition(closedClawDeg);
        }
    }
    public void doManualForearm(Gamepad gamepad2) {
        if (gamepad2.a) {
            robot.forearmServoL.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.forearmServoL.setPower(1);
            robot.forearmServoR.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.forearmServoR.setPower(1);
        }
        if (gamepad2.b) {
            robot.forearmServoL.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.forearmServoL.setPower(1);
            robot.forearmServoR.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.forearmServoR.setPower(1);
        }
        if (gamepad2.x) {
            robot.forearmServoL.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.forearmServoL.setPower(0);
            robot.forearmServoR.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.forearmServoR.setPower(0);
        }
    }

    public int angleToTicks(double x) {
        return (int) (x * EncoderTPR / 360);
    }
    public void goToForearm(double targetAngle, double rate) { //Assumes 0 is set to vertical arm position
        if(angleToTicks(targetAngle) > robot.forearmEncoder.getCurrentPosition()) { //if target is ahead of current
            robot.forearmServoL.setPower(rate);
            robot.forearmServoR.setPower(rate);
        } else {
            robot.forearmServoL.setPower(-rate);
            robot.forearmServoR.setPower(-rate);
        }
    }
    /*
    public void doTestForearm(Gamepad gamepad2, boolean allowPastEndstops) {
        int newTargetPosition;
        if (gamepad2.a) {
            newTargetPosition = 170;
            if (!allowPastEndstops) {
                newTargetPosition = (int) clamp(newTargetPosition, FOREARM_MIN, FOREARM_MAX);
                robot.forearmMotor.setTargetPosition(newTargetPosition);
            }
        }
        if (gamepad2.b) {
            newTargetPosition = 0;
            if (!allowPastEndstops) {
                newTargetPosition = (int) clamp(newTargetPosition, FOREARM_MIN, FOREARM_MAX);
                robot.forearmMotor.setTargetPosition(newTargetPosition);
            }
        }
    }
*/
    public void goToLinear(int newTargetPosition, double speed) {
        if(newTargetPosition != robot.linearExtenderMotorL.getTargetPosition() && newTargetPosition != robot.linearExtenderMotorR.getTargetPosition()) {
            newTargetPosition = (int) clamp(newTargetPosition, LINEAR_MIN, LINEAR_MAX);
            robot.linearExtenderMotorL.setPower(speed);
            robot.linearExtenderMotorR.setPower(speed);
            robot.linearRetractor.setPower(speed);
            robot.linearExtenderMotorL.setTargetPosition(newTargetPosition);
            robot.linearExtenderMotorR.setTargetPosition(newTargetPosition);
            robot.linearRetractor.setTargetPosition(newTargetPosition);
        }
    }
    public void doManualLinear(Gamepad gamepad2, boolean allowPastEndstops){
        int newTargetPosition = robot.linearExtenderMotorL.getTargetPosition();
        if (Math.abs(gamepad2.left_stick_y) > .15){
            newTargetPosition += -40 * gamepad2.left_stick_y; // negative 40 because y is reversed
        }
        if(!allowPastEndstops) {
            newTargetPosition = (int) clamp(newTargetPosition, LINEAR_MIN, LINEAR_MAX);
        }
        robot.linearRetractor.setTargetPosition(newTargetPosition);
        robot.linearExtenderMotorL.setTargetPosition(newTargetPosition);
        robot.linearExtenderMotorR.setTargetPosition(newTargetPosition);
    }
    private double clamp(double val, double min, double max){
        return Math.max(min, Math.min(max, val));
    }
}

