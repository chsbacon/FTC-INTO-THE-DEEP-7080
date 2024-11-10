package org.firstinspires.ftc.teamcode.drive.modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmController {
    public enum ArmLocation {
        Intake,
        LowBasket,
        HighBasket,
        LowSub,
        HighSub,
        Hang
    }
    private Robot2024 robot;
    private Telemetry telemetry;
    public final int LINEAR_MIN = 0;
    public final int LINEAR_MAX = 2500;
    public final int FOREARM_MIN = 0;
    public final int FOREARM_MAX = 2*90;
    private ElapsedTime loopTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private boolean clawOpen = false;
    private final int openClawDeg = 30; // need to adjust these in testing
    private final int closedClawDeg = -5;
    public void onOpmodeInit(Robot2024 robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
        robot.linearExtenderMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.linearExtenderMotorL.setTargetPosition(0);
        robot.linearExtenderMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.linearExtenderMotorL.setPower(1);

        robot.linearExtenderMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.linearExtenderMotorR.setTargetPosition(0);
        robot.linearExtenderMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.linearExtenderMotorR.setPower(1);

        robot.forearmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.forearmMotor.setTargetPosition(0);
        robot.forearmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.forearmMotor.setPower(1);
    }
    public void doLoop(Gamepad gamepad1, Gamepad gamepad2){

        doManualLinear(gamepad2, gamepad2.start && gamepad2.left_bumper);
        doManualForearm(gamepad2, gamepad2.start && gamepad2.left_bumper);
        doClawControl(gamepad2);

        /*
        if(gamepad2.a){
            armTargetLocation = ArmLocation.Intake;
            actionExecutor.setAction(new ParallelAction(
                    //goToLinearHeightAction(LINEAR_MAX),
                    goToArmPositionAction(FOREARM_MIN),
                    //new SleepAction(0.5),
                    goToLinearHeightAction(LINEAR_MIN)
            ));
        }
        if(gamepad2.x){
            armTargetLocation = ArmLocation.Hang;
            actionExecutor.setAction(new ParallelAction(
                    goToLinearHeightAction(LINEAR_MAX),
                    goToArmPositionAction(FOREARM_VERTICAL)
            ));
        }
        if(gamepad2.b && !gamepad2.dpad_left){
            armTargetLocation = ArmLocation.Score;
            actionExecutor.setAction(new ParallelAction(
                    //goToLinearHeightAction(LINEAR_MAX),
                    goToArmPositionAction(800),
                    goToLinearHeightAction(LINEAR_MIN)
            ));
        }
        if(gamepad2.y){
            armTargetLocation = ArmLocation.Score;
            actionExecutor.setAction(new ParallelAction(
                    //goToLinearHeightAction(LINEAR_MAX),
                    goToArmPositionAction(175),
                    goToLinearHeightAction(LINEAR_MAX)
            ));
        }
        if(gamepad2.back && gamepad2.start){
            for(DcMotorEx motor: new DcMotorEx[]{robot.leftForearmMotor, robot.rightForearmMotor}){
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            forearmPID.reset();
            forearmPID.setSetPoint(0);
            robot.linearExtenderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.linearExtenderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.linearExtenderMotor.setTargetPosition(0);
        }


        doArmControl();

         */
        if(gamepad2.back){
            for(DcMotorEx motor: new DcMotorEx[]{robot.linearExtenderMotorL,robot.linearExtenderMotorR}){
                motor.setPower(0);
            }
        } else {
            robot.linearExtenderMotorL.setPower(1);
            robot.linearExtenderMotorR.setPower(1);
        }

        telemetry.addData("left: LinearC: ", robot.linearExtenderMotorL.getCurrentPosition());
        telemetry.addData("right: LinearC: ", robot.linearExtenderMotorR.getCurrentPosition());
        telemetry.addData("left linearT: ", robot.linearExtenderMotorL.getTargetPosition());
        telemetry.addData("left linearT: ", robot.linearExtenderMotorL.getTargetPosition());
        loopTimer.reset();
    }

    public void doClawControl(Gamepad gamepad2){

        // this abstraction seems pointless from an efficentcy standpoint why not run the check in the main loop?
        if (gamepad2.left_bumper) {
            clawOpen = !clawOpen;
            if (clawOpen) {
                robot.clawServo.setPosition(openClawDeg);
            } else {
                robot.clawServo.setPosition(closedClawDeg);
            }
        }
    }

    public void doManualForearm(Gamepad gamepad2, boolean allowPastEndstops) {
        int newTargetPosition = robot.forearmMotor.getTargetPosition();
        if (gamepad2.a) {
            newTargetPosition += 2;
        }
        if (gamepad2.b) {
            newTargetPosition -= 2;
        }
        if (!allowPastEndstops) {
            newTargetPosition = (int) clamp(newTargetPosition, FOREARM_MIN, FOREARM_MAX);
        }
        robot.forearmMotor.setTargetPosition(newTargetPosition);
    }

    public void doManualLinear(Gamepad gamepad2, boolean allowPastEndstops){
        int newTargetPosition = robot.linearExtenderMotorL.getTargetPosition();
        if (Math.abs(gamepad2.left_stick_y) > .15){
            newTargetPosition += -40 * gamepad2.left_stick_y; // negative 40 because y is reversed
        }
        if(!allowPastEndstops) {
            newTargetPosition = (int) clamp(newTargetPosition, LINEAR_MIN, LINEAR_MAX);
        }
        robot.linearExtenderMotorL.setTargetPosition(newTargetPosition);
        robot.linearExtenderMotorR.setTargetPosition(newTargetPosition);
    }
    private double clamp(double val, double min, double max){
        return Math.max(min, Math.min(max, val));
    }
}

