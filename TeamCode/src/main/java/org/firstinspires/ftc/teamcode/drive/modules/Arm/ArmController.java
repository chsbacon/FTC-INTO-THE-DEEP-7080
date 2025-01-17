package org.firstinspires.ftc.teamcode.drive.modules.Arm;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.hardware.MecanumDrive2024;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.modules.Robot2024;

public class ArmController {
    private Robot2024 robot;
    private MecanumDrive2024 drive;
    private Telemetry telemetry;
    private ElapsedTime loopTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private boolean inDebugMode = false;
    private ArmState currentState;
    private boolean prevTrigger=false;



    public final int ServoTPR = 0;


    // Motor consts

    private static final double EXTENDER_LSLIDE_MOTOR_TPR = 537.7; // go bilda
    private static final double ROTATOR_MOTOR_TPR = 1527.79; // Rev motor

    public static final int MAX_EXTEND = 3600;


    private static int FOREARM_HORIZ_TICK = 648;
    private static int FOREARM_VERT_TICK = 0;

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
        telemetry.addData("Claw Position", robot.clawServo.getPosition());
        telemetry.addData("Twist Position", robot.twistServo.getPosition());
        telemetry.addData("Wrist Position", robot.wristServo.getPosition());
        telemetry.update();

        if (gamepad2.right_bumper) {
            moveClaw(true);
        }
        else if (gamepad2.left_bumper) {
            moveClaw(false);
        }
/*
        if (gamepad2.right_bumper) {
            moveWristToAngle(0);
        }
        else if (gamepad2.left_bumper) {
            moveWristToAngle(0);
        }

 */

        double wristTargetPosition = robot.wristServo.getPosition();
        if (gamepad2.dpad_up) {
            wristTargetPosition += 1.0/270; //1 degree?
        }
        else if (gamepad2.dpad_down) {
            wristTargetPosition -= 1.0/270;
        }
        robot.wristServo.setPosition(wristTargetPosition);

        double twistTargetPosition = robot.twistServo.getPosition();
        twistTargetPosition += gamepad2.right_stick_x/100;
        robot.twistServo.setPosition(twistTargetPosition);

        double slideTargetPosition = robot.linearExtenderMotorL.getCurrentPosition();
        slideTargetPosition += gamepad1.right_stick_y;
        manualExtension(gamepad2);
        /*if(gamepad2.dpad_left) {
            rotateSlide(false,0.5,false);
        }
        if(gamepad2.dpad_right) {
            rotateSlide(true,0.5,false);
        }
         */
        if (gamepad2.b) {
         setCurrentState(new Neutral());
         move();
        }
        else if (gamepad2.y) {
            setCurrentState(new UpperBucketDeposit());
            move();
        }
        else if (gamepad2.x) {
            setCurrentState(new HighChamberDeposit());
            move();
        }
        else if (gamepad2.a) {
            setCurrentState(new GroundIntake());
            move();
        }
        /*
        if  (gamepad2.a && gamepad2.left_bumper){
            setCurrentState(new WallIntake());
            move();
        }

         */


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
        if (currentState != null){
            currentState.move(this);
        }
        currentState = state;
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

    public ArmState getCurrentState(){
        return currentState;
    }

    public void manualRotation(Gamepad gamepad2) {
        int newTargetPosition=robot.armRotationMotorL.getCurrentPosition();
        if(gamepad2.a&&gamepad2.right_bumper) {
            newTargetPosition+=60;
        } else if(gamepad2.b&&gamepad2.right_bumper) {
            newTargetPosition-=60;
        }
        robot.armRotationMotorL.setTargetPosition(newTargetPosition);
        robot.armRotationMotorR.setTargetPosition(newTargetPosition);
    }
    public void manualExtension(Gamepad gamepad2) {
        double speedModifier = 1;


        if (Math.abs(gamepad2.left_stick_y) > .15) { //if stick is pressed far enough
            for (DcMotorEx motor : new DcMotorEx[]{robot.linearExtenderMotorL, robot.linearExtenderMotorR}) { //set extenders to run on power
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if((robot.linearExtenderMotorL.getCurrentPosition()>20
                        &&gamepad2.left_stick_y<0)
                        &&(robot.linearExtenderMotorL.getCurrentPosition()<MAX_EXTEND)||(robot.linearExtenderMotorR.getCurrentPosition()<MAX_EXTEND)) { //Don't break the slide
                    if (gamepad2.left_trigger > 0.15){
                        motor.setPower(-gamepad2.left_stick_y * 0.5); //power proportional to joystick;
                    } else {
                        motor.setPower(-gamepad2.left_stick_y); //power proportional to joystick;
                    }
                } else {
                    motor.setPower(0);
                }
            }
            prevTrigger=true;
        } else if(prevTrigger&&Math.abs(gamepad2.left_stick_y) < .15){ //if stick is not pressed enough
            for(DcMotorEx motor: new DcMotorEx[]{robot.linearExtenderMotorL,robot.linearExtenderMotorR}){ //reset extenders to run by encoder
                motor.setTargetPosition(robot.linearExtenderMotorL.getCurrentPosition());
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setPower(0.5 * speedModifier);
                motor.setTargetPosition(robot.linearExtenderMotorL.getCurrentPosition());
            }
            prevTrigger=false;
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

    public void twistClawToAngle(double angle){
        final int TWIST_MAX_ANGLE = 0;
        final int TWIST_MIN_ANGLE = 0;
        // TODO: 1/16/25 Measure values
        robot.twistServo.setPosition(angle);
    }

    public void twistClaw(double tick){
        robot.twistServo.setPosition(tick);
    }

    public void moveWristToAngle(double wristAngle) {
        final double WRIST_MAX_ANGLE = 1;
        final double WRIST_MIN_ANGLE = 0;
        final double  WRIST_0_OFFSET = 0; // TODO: 1/16/25 Measure and Mark
        robot.wristServo.setPosition(wristAngle);
    }

    public void moveClaw(double angle){
        final double CLAW_MAX_ANGLE = 1;
        final double CLAW_MIN_ANGLE = 0;
        // TODO: 1/16/25 Measure values
        robot.clawServo.setPosition(angle);
    }


    //Implementing this is counter intuitive, as its not clear that passing a bool will make the claw open or close, maybe rename?
    public void moveClaw(boolean open){
        final double CLAW_OPEN_ANGLE = 0;
        final double CLAW_CLOSED_ANGLE = 0.4; // TODO: 1/16/25 Measure and Mark
        if(open){
            moveClaw(CLAW_OPEN_ANGLE);
        } else {
            moveClaw(CLAW_CLOSED_ANGLE);
        }
    }



    public void rotateSlide(int tick, double speed, boolean delay){
        // TODO: Measure actual values and mark after tests
        robot.armRotationMotorL.setTargetPosition((int) clamp(tick, FOREARM_VERT_TICK, FOREARM_HORIZ_TICK));
        robot.armRotationMotorR.setTargetPosition((int) clamp(tick, FOREARM_VERT_TICK, FOREARM_HORIZ_TICK));
        robot.armRotationMotorL.setPower(speed);
        robot.armRotationMotorR.setPower(speed);
        if(delay) {
            while (tick != robot.armRotationMotorL.getCurrentPosition()) {
            } //wait until slide arrives to position
        }
    }

    public void rotateSlide(boolean isVertical, double speed, boolean delay){
        rotateSlide(isVertical ? FOREARM_VERT_TICK : FOREARM_HORIZ_TICK, speed, delay);
    }

    public void extendSlideToTick(int tick, double speed,boolean delay){
        final int EXTENDER_LSLIDE_MOTOR_MIN_TICK = 20;
        final int EXTENDER_LSLIDE_MOTOR_MAX_TICK = MAX_EXTEND; // TODO: Get the right max value

        robot.linearExtenderMotorL.setTargetPosition((int)clamp(tick, EXTENDER_LSLIDE_MOTOR_MIN_TICK, EXTENDER_LSLIDE_MOTOR_MAX_TICK));
        robot.linearExtenderMotorR.setTargetPosition((int)clamp(tick, EXTENDER_LSLIDE_MOTOR_MIN_TICK, EXTENDER_LSLIDE_MOTOR_MAX_TICK));
        robot.linearExtenderMotorL.setPower(speed);
        robot.linearExtenderMotorR.setPower(speed);
        if(delay) {
            while(tick!=robot.linearExtenderMotorL.getCurrentPosition()) {
            } //wait until slide arrives to position
        }
    }

    public void extendSlideToAngle(double angle, int speed){
        extendSlideToTick(angleToTicks(angle,true),speed,false);
    }
    private double clamp(double val, double min, double max){
        return Math.max(min, Math.min(max, val));
    }

    //straight rip from arduino
    private double mapRange(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }




}

