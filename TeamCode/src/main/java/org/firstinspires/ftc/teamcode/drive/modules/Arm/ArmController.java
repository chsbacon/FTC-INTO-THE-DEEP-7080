package org.firstinspires.ftc.teamcode.drive.modules.Arm;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.hardware.MecanumDrive2024;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.modules.Robot2024;

import org.firstinspires.ftc.teamcode.drive.modules.Arm.*;

public class ArmController {
    private Robot2024 robot;
    private MecanumDrive2024 drive;
    private Telemetry telemetry;
    private ElapsedTime loopTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


    private ArmState currentState;

    // Linear slide constants
    public final int EXTENDER_LSLIDE_MOTOR_MIN_TICK = 0;
    public final int EXTENDER_LSLIDE_MOTOR_MAX_TICK = 5580; // TODO: Get the right max value

    // Motor consts

    private final int EXTENDER_LSLIDE_MOTOR_TPR = 8192; // TODO: uses old value, reaffirm with new measurements
    private final int ROTATOR_MOTOR_TPR = 8192; // TODO: New motor, need to measure
// Extender motor is go bilda and rotation motor is rev core hex (TPR =Ticks per rotation)

    // Forearm rotation constants
    // TODO: Measure actual values and mark after tests

    public final double FOREARM_VERT_ENCODER_TICK = 0; //
    private final double FOREARM_HORIZ_ENCODER_TICK = 0; //
    // Probobly should move these into method variables when we implement the arm extension method

    public void onOpmodeInit(Robot2024 robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
        for(DcMotorEx motor: new DcMotorEx[]{robot.linearExtenderMotorL,robot.linearExtenderMotorR}){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(0);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(1);
        }


    }
    public void doLoop(Gamepad gamepad1, Gamepad gamepad2){

        telemetry.addData("left: linearC: ", robot.linearExtenderMotorL.getCurrentPosition());
        telemetry.addData("right: linearC: ", robot.linearExtenderMotorR.getCurrentPosition());
        telemetry.addData("left: linearT: ", robot.linearExtenderMotorL.getTargetPosition());
        telemetry.addData("right: linearT: ", robot.linearExtenderMotorL.getTargetPosition());
        telemetry.addData("encoder position", robot.forearmEncoder.getCurrentPosition());
        loopTimer.reset();
    }

    public void move(){
        if (currentState != null){
            currentState.move(this);
        }
    }

    public void move(ArmState state){
        currentState = state;
        if (currentState != null){
            currentState.move(this);
        }
    }

    public void setCurrentState(ArmState newState){
        currentState = newState;
    }

    /**
     * Converts an angle to ticks for the 2 DC motors that control the linear slide extending and the rotating of the forearm
     * @param theta the angle to convert
     * @param isExtenderMotor true if the motor is the linear slide extender, false if it is the forearm rotator
     * @return Returns the number of ticks to move the motor
     */

    public int angleToTicks(double theta, boolean isExtenderMotor) {
        if (isExtenderMotor) {
            return (int) (theta * EXTENDER_LSLIDE_MOTOR_TPR / 360);
        }
        return (int) (theta * ROTATOR_MOTOR_TPR / 360);
    }

    @Deprecated
    public void goToForearm(double targetAngle, double rate) { //Assumes 0 is set to vertical arm position

//        if(angleToTicks(targetAngle,true) > robot.forearmEncoder.getCurrentPosition()) { //if target is ahead of current
//            robot.forearmServoL.setPower(rate);
//            robot.forearmServoR.setPower(rate);
//        } else {
//            robot.forearmServoL.setPower(-rate);
//            robot.forearmServoR.setPower(-rate);
//        }
        throw new UnsupportedOperationException("This method is deprecated");
    }

    public void moveClaw(int clawAngle, double wristAngle){
         final int CLAW_MAX_ANGLE = 180;
         final int CLAW_MIN_ANGLE = 0;
        // TODO: 1/15/25 measure these values
        robot.clawServo.setPosition(clamp(clawAngle,CLAW_MIN_ANGLE,CLAW_MAX_ANGLE) );

    }

    @Deprecated
    public void goToLinear(int newTargetPosition, double speed) {

        if(newTargetPosition != robot.linearExtenderMotorL.getTargetPosition() && newTargetPosition != robot.linearExtenderMotorR.getTargetPosition()) {
            newTargetPosition = (int) clamp(newTargetPosition, EXTENDER_LSLIDE_MOTOR_MIN_TICK, EXTENDER_LSLIDE_MOTOR_MAX_TICK);
//            robot.linearExtenderMotorL.setPower(speed);
//            robot.linearExtenderMotorR.setPower(speed);
//            robot.linearRetractor.setPower(speed);
//            robot.linearExtenderMotorL.setTargetPosition(newTargetPosition);
//            robot.linearExtenderMotorR.setTargetPosition(newTargetPosition);
//            robot.linearRetractor.setTargetPosition(newTargetPosition);
        }
        throw new UnsupportedOperationException("This method is deprecated");
    }

    @Deprecated
    public void doManualLinear(Gamepad gamepad2, boolean allowPastEndstops){
        int newTargetPosition = robot.linearExtenderMotorL.getTargetPosition();
        if (Math.abs(gamepad2.left_stick_y) > .15){
            newTargetPosition += -40 * gamepad2.left_stick_y; // negative 40 because y is reversed
        }
        if(!allowPastEndstops) {
            newTargetPosition = (int) clamp(newTargetPosition, EXTENDER_LSLIDE_MOTOR_MIN_TICK, EXTENDER_LSLIDE_MOTOR_MAX_TICK);
        }
        robot.linearRetractor.setTargetPosition(newTargetPosition);
        robot.linearExtenderMotorL.setTargetPosition(newTargetPosition);
        robot.linearExtenderMotorR.setTargetPosition(newTargetPosition);
        throw new UnsupportedOperationException("This method is deprecated");
    }

    private double clamp(double val, double min, double max){
        return Math.max(min, Math.min(max, val));
    }
}

