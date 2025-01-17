package org.firstinspires.ftc.teamcode.drive.modules.Arm;

public class GroundIntake implements ArmState {

    private static final int GROUND_EXTENDER_ENCODER_TICK = 700;
    private static final double WRIST_INTAKE_ANGLE = .79;
    ;


    @Override
    public void move(ArmController armController) {
        armController.extendSlideToTick(GROUND_EXTENDER_ENCODER_TICK, .5,true); //Wait for unstick
        armController.rotateSlide(false, .5,false);
        armController.moveWristToAngle(WRIST_INTAKE_ANGLE);
        armController.moveClaw(false);
        armController.twistClawToAngle(.13); // TODO: 1/14/25 mesure
    }
}
