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
    private boolean inDebugMode = false;
    private ArmState currentState;



    public final int ServoTPR = 0;


    // Motor consts

    private static final double EXTENDER_LSLIDE_MOTOR_TPR = 537.7; // go bilda
    private static final double ROTATOR_MOTOR_TPR = 1527.79; // Rev motor



// Extender motor is go bilda and rotation motor is rev core hex (TPR =Ticks per rotation)


    // Probobly should move these into method variables when we implement the arm extension method

    public void onOpmodeInit(Robot2024 robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
        for(DcMotorEx motor: new DcMotorEx[]{robot.linearExtenderMotorL,robot.linearExtenderMotorR,robot.armRotationMotorR,robot.armRotationMotorL}){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(0);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(0.5);
        }


    }
    public void doLoop(Gamepad gamepad1, Gamepad gamepad2){
        if(inDebugMode){
            debugDoLoop(gamepad1, gamepad2);
            return;
        }

        telemetry.addData("Extender Motor L", robot.linearExtenderMotorL.getCurrentPosition());
        telemetry.addData("Extender Motor R", robot.linearExtenderMotorR.getCurrentPosition());
        telemetry.addData("Rotator Motor L", robot.armRotationMotorL.getCurrentPosition());
        telemetry.addData("Rotator Motor R", robot.armRotationMotorR.getCurrentPosition());
        telemetry.addData("Wrist Position", robot.wristServo.getPosition());

        if (gamepad2.right_trigger >= 0.5) {
            moveClaw(true);
        }
        else if (gamepad2.left_trigger >= 0.5) {
            moveClaw(false);
        }

        if (gamepad2.right_bumper) {
            moveWristToAngle(0);
        }
        else if (gamepad2.left_bumper) {
            moveWristToAngle(0);
        }

        double wristTargetPosition = robot.wristServo.getPosition();
        if (gamepad2.dpad_up) {
            wristTargetPosition += 10;
        }
        else if (gamepad2.dpad_down) {
            wristTargetPosition -= 10;
        }
        robot.wristServo.setPosition(wristTargetPosition);

        double twistTargetPosition = robot.twistServo.getPosition();
        twistTargetPosition += gamepad1.left_stick_x;
        robot.twistServo.setPosition(twistTargetPosition);

        double slideTargetPosition = robot.linearExtenderMotorL.getCurrentPosition();
        slideTargetPosition += gamepad1.right_stick_y;

        if (gamepad2.b) {
         setCurrentState(new Neutral());
        }
        else if (gamepad2.y) {
            setCurrentState(new UpperBucketDeposit());
        }
        else if (gamepad2.x) {
            setCurrentState(new HighChamberDeposit());
        }
        else if (gamepad2.a) {
            setCurrentState(new GroundIntake());
        }
        /*

        - implement things from MD file here

         */

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

    public void enableDebugMode(boolean enable){
        inDebugMode = enable;
    }

    public void debugDoLoop(Gamepad gamepad1, Gamepad gamepad2){
        manualRotation(gamepad2);
        manualExtension(gamepad2);

        telemetry.addData("Extender Motor L", robot.linearExtenderMotorL.getCurrentPosition());
        telemetry.addData("Extender Motor R", robot.linearExtenderMotorR.getCurrentPosition());
        telemetry.addData("Rotator Motor L", robot.armRotationMotorL.getCurrentPosition());
        telemetry.addData("Rotator Motor R", robot.armRotationMotorR.getCurrentPosition());
        telemetry.addData("Wrist Position", robot.wristServo.getPosition());
        telemetry.update();


        // TODO: 1/16/25 implement manual controls
    }

    public void setCurrentState(ArmState newState){
        currentState = newState;
    }

    public void manualRotation(Gamepad gamepad2) {
        int newTargetPosition=robot.armRotationMotorL.getCurrentPosition();
        if(gamepad2.a) {
            newTargetPosition+=10;
        } else if(gamepad2.b) {
            newTargetPosition-=10;
        }
        robot.armRotationMotorL.setTargetPosition(newTargetPosition);
        robot.armRotationMotorR.setTargetPosition(newTargetPosition);
    }
    public void manualExtension(Gamepad gamepad2) {
        if (Math.abs(gamepad2.left_stick_y) > .15) { //if stick is pressed far enough
            for (DcMotorEx motor : new DcMotorEx[]{robot.linearExtenderMotorL, robot.linearExtenderMotorR}) { //set extenders to run on power
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor.setPower(gamepad2.left_stick_y); //power proportional to joystick
            }
        } else { //if stick is not pressed enough
            for(DcMotorEx motor: new DcMotorEx[]{robot.linearExtenderMotorL,robot.linearExtenderMotorR}){ //reset extenders to run by encoder
                motor.setTargetPosition(robot.linearExtenderMotorL.getCurrentPosition());
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setPower(0.5);
            }
        }
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





    public void twistClawToAngle(int angle){
        final int TWSIT_MAX_ANGLE = 0;
        final int TWSIT_MIN_ANGLE = 0;
        // TODO: 1/16/25 Measure values
        robot.clawServo.setPosition(mapRange(clamp(angle,TWSIT_MIN_ANGLE,TWSIT_MAX_ANGLE),0,180,0,1));
    }

    public void moveWristToAngle(double wristAngle) {
        final double WRIST_MAX_ANGLE = 180;
        final double WRIST_MIN_ANGLE = 0;
        robot.wristServo.setPosition(clamp(wristAngle, WRIST_MIN_ANGLE, WRIST_MAX_ANGLE));
    }

    public void moveClaw(int angle){
        final int CLAW_MAX_ANGLE = 0;
        final int CLAW_MIN_ANGLE = 0;
        // TODO: 1/16/25 Measure values
        robot.clawServo.setPosition(mapRange(clamp(angle,CLAW_MIN_ANGLE,CLAW_MAX_ANGLE),0,180,0,1));
    }


    //Implementing this is counter intuitive, as its not clear that passing a bool will make the claw open or close, maybe rename?
    public void moveClaw(boolean open){
        final int CLAW_OPEN_ANGLE = 0;
        final int CLAW_CLOSED_ANGLE = 0; // TODO: 1/16/25 Measure and Mark
        if(open){
            moveClaw(CLAW_OPEN_ANGLE);
        } else {
            moveClaw(CLAW_CLOSED_ANGLE);
        }
    }



    public void rotateSlideToTick(int tick, int speed){
        // TODO: Measure actual values and mark after tests
        final int FOREARM_HORIZ_TICK = 0;
        final int FOREARM_VERT_TICK = 0;
        robot.armRotationMotorL.setTargetPosition((int) clamp(tick, FOREARM_HORIZ_TICK, FOREARM_VERT_TICK));
        robot.armRotationMotorR.setTargetPosition((int) clamp(tick, FOREARM_HORIZ_TICK, FOREARM_VERT_TICK));
        robot.armRotationMotorL.setPower(speed);
        robot.armRotationMotorR.setPower(speed);
    }

    public void rotateSlideToAngle(double angle,int speed){
        rotateSlideToTick(angleToTicks(angle,false),speed);
    }

    public void extendSlideToTick(int tick, int speed){
        final int EXTENDER_LSLIDE_MOTOR_MIN_TICK = 0;
        final int EXTENDER_LSLIDE_MOTOR_MAX_TICK = 0; // TODO: Get the right max value

        robot.linearExtenderMotorL.setTargetPosition((int)clamp(tick, EXTENDER_LSLIDE_MOTOR_MIN_TICK, EXTENDER_LSLIDE_MOTOR_MAX_TICK));
        robot.linearExtenderMotorR.setTargetPosition((int)clamp(tick, EXTENDER_LSLIDE_MOTOR_MIN_TICK, EXTENDER_LSLIDE_MOTOR_MAX_TICK));
        robot.linearExtenderMotorL.setPower(speed);
        robot.linearExtenderMotorR.setPower(speed);
    }

    public void extendSlideToAngle(double angle, int speed){
        extendSlideToTick(angleToTicks(angle,true),speed);
    }

    private double clamp(double val, double min, double max){
        return Math.max(min, Math.min(max, val));
    }

    //straight rip from arduino
    private double mapRange(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    @Deprecated
    public void goToLinear(int newTargetPosition, double speed) {

        if(newTargetPosition != robot.linearExtenderMotorL.getTargetPosition() && newTargetPosition != robot.linearExtenderMotorR.getTargetPosition()) {
//            newTargetPosition = (int) clamp(newTargetPosition, EXTENDER_LSLIDE_MOTOR_MIN_TICK, EXTENDER_LSLIDE_MOTOR_MAX_TICK);
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
//        int newTargetPosition = robot.linearExtenderMotorL.getTargetPosition();
//        if (Math.abs(gamepad2.left_stick_y) > .15){
//            newTargetPosition += -40 * gamepad2.left_stick_y; // negative 40 because y is reversed
//        }
//        if(!allowPastEndstops) {
//            newTargetPosition = (int) clamp(newTargetPosition, EXTENDER_LSLIDE_MOTOR_MIN_TICK, EXTENDER_LSLIDE_MOTOR_MAX_TICK);
//        }
//        robot.linearRetractor.setTargetPosition(newTargetPosition);
//        robot.linearExtenderMotorL.setTargetPosition(newTargetPosition);
//        robot.linearExtenderMotorR.setTargetPosition(newTargetPosition);
        throw new UnsupportedOperationException("This method is deprecated");
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



}

