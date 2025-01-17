package org.firstinspires.ftc.teamcode.drive.modules.Arm;

public class UpperBucketDeposit implements ArmState {

    private static final int UNSTICK = 700;
    private static final int HIGH_BASKET_EXTENDER_ENCODER_TICK = 3550;

    @Override
    public void move(ArmController armController) {
        //        TODO: Implement this method
        armController.extendSlideToTick(UNSTICK, 0.5,true);//Wait till unstuck
        armController.rotateSlide(true, .5,false);//Rotate
        armController.extendSlideToTick(HIGH_BASKET_EXTENDER_ENCODER_TICK, 0.5,false);//Extend remaining
        armController.moveClaw(true);
        armController.moveWristToAngle(  0.28);
        armController.twistClaw(.889); //low
    }
}
