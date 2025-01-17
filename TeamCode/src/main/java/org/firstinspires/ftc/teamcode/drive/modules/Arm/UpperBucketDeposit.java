package org.firstinspires.ftc.teamcode.drive.modules.Arm;

public class UpperBucketDeposit implements ArmState {

    private static final int HIGH_BASKET_EXTENDER_ENCODER_TICK = 0;

    @Override
    public void move(ArmController armController) {
        //        TODO: Implement this method
        armController.extendSlideToTick(HIGH_BASKET_EXTENDER_ENCODER_TICK, 0);
        armController.rotateSlideToAngle(0, 0);
    }
}
