package org.firstinspires.ftc.teamcode.drive.modules.Arm;

public class GroundIntake implements ArmState {

    private static final int GROUND_EXTENDER_ENCODER_TICK = 0;
    private static final int INTAKE_ROTATION_ENCODER_TICK = 0;


    @Override
    public void move(ArmController armController) {
        armController.extendSlideToTick(GROUND_EXTENDER_ENCODER_TICK, 0);
        armController.rotateSlideToAngle(INTAKE_ROTATION_ENCODER_TICK, 0);
        armController.moveWristToAngle(0);
    }
}
