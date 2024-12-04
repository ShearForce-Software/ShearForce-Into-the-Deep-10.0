package org.firstinspires.ftc.teamcode.Geronimo;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Geronimo 1 Manual Control")
//@Disabled
public class Geronimo_Manual_Control extends LinearOpMode {
    Geronimo theRobot;
    boolean rotatorPowerApplied = false;

    // Scrimmage Meet Ideas:
    // Press intake button, have claw grab the sample
    // Have claw go down when slides extend forward?
    // 1 button to grab specimen off wall (rotate slides, Limelight correction, grab, rotate claw up to get off wall)

    public void runOpMode() {
        theRobot = new Geronimo(true, false, this);

        int rotatorSetPosition = 0;

        theRobot.Init(this.hardwareMap);
        theRobot.ShowTelemetry();

        telemetry.update();
        waitForStart();
        resetRuntime();
        theRobot.intakeStar.setPower(1.0);

            while (opModeIsActive()) {
                theRobot.EndgameBuzzer();
                /* *************************************************
                 *************************************************
                 * Driver Controls (gamepad1)
                 *************************************************
                 *************************************************
                 */
                // Drive Controls uses left_stick_y, left_stick_x, and right_stick_x
                theRobot.driveControlsFieldCentric();
                //theRobot.driveControlsRobotCentric();

                if(gamepad1.y){
                    theRobot.imu.resetYaw();
                }

                /* *************************************************
                 *************************************************
                 * Gunner / Arm Controls (gamepad2)
                 *
                 *************************************************
                 *************************************************
                 */
                // SLIDE MOTOR CONTROL
                if ((gamepad2.left_stick_y > 0.1) || (gamepad2.left_stick_y <= -0.1)) {
                    theRobot.SetSlidesPower(gamepad2.left_stick_y);
                }
                else
                {
                    theRobot.SetSlidesPower(0.0);
                }

                // RightStickOne Driver 2
                if(gamepad2.right_stick_y > 0.25){
                    rotatorPowerApplied = true;
                    theRobot.SetRotatorArmPower(0.4);
                }
                else if(gamepad2.right_stick_y <= -0.25){
                    rotatorPowerApplied = true;
                    theRobot.SetRotatorArmPower(-0.4);
                }
                else if (rotatorPowerApplied && !theRobot.GetRotatorArmRunningToPosition() && !theRobot.GetRotatorLimitSwitchPressed()) {
                    rotatorPowerApplied = false;
                    theRobot.SetRotatorToPosition(theRobot.GetRotatorLeftArmCurrentPosition());
                 //  theRobot.SetRotatorArmHoldPosition();
                }

                // gamepad_1.square --> stop and reset on the slides
                if (gamepad1.square) {
                    theRobot.ResetSlidesPower();
                }
/*
                // Rotater MOTOR CONTROL
                if ((gamepad2.right_stick_y > 0.1) || (gamepad2.right_stick_y <= -0.1)) {
                    //theRobot.SetRotatorArmPower(gamepad2.right_stick_y);
                    rotatorSetPosition = theRobot.GetRotatorArmTargetPosition();
                    rotatorSetPosition += Math.round(gamepad2.right_stick_y * 10);
                    theRobot.SetRotatorToPosition(rotatorSetPosition);
                }
                else if (gamepad2.circle && !gamepad2.options) {
                    theRobot.SetRotatorToPosition(250);
                }
                else if (gamepad2.triangle) {
                    theRobot.SetRotatorToPosition(450);
                }
                else if (gamepad2.square) {
                    theRobot.SetRotatorToPosition(820);
                }
                else if (gamepad2.cross && !gamepad2.options) {
                    theRobot.SetRotatorToPosition(0);
                }
                else if (theRobot.GetRotatorArmRunningToPosition())
                {
                    int leftDifference = Math.abs(theRobot.GetRotatorArmTargetPosition() - theRobot.GetRotatorLeftArmCurrentPosition());
                    int rightDifference = Math.abs(theRobot.GetRotatorArmTargetPosition() - theRobot.GetRotatorRightArmCurrentPosition());
                    if ((leftDifference <= 2) || (rightDifference <= 2))
                    {
                        theRobot.SetRotatorArmHoldPositon();
                    }
                }
                //else
                //{
                //    theRobot.SetRotatorArmPower(0.0);
                //}

 */
                if (theRobot.GetRotatorArmRunningToPosition()) {
                    if (theRobot.GetRotatorArmTargetPosition() == 0 && theRobot.GetRotatorLimitSwitchPressed() == true){
                        theRobot.setRotatorArmZeroize();

                    }
                    else{
                        int leftDifference = Math.abs(theRobot.GetRotatorArmTargetPosition() - theRobot.GetRotatorLeftArmCurrentPosition());
                        int rightDifference = Math.abs(theRobot.GetRotatorArmTargetPosition() - theRobot.GetRotatorRightArmCurrentPosition());
                        if ((leftDifference <= 2) && (rightDifference <= 2)) {
                            theRobot.SetRotatorArmHoldPosition();
                        }
                    }
                }

                if (gamepad2.cross && !gamepad2.options) {
                    theRobot.IntakeFromFloor();
                }
                else if (gamepad2.share){
                    theRobot.AutoStartPosition();
                }
             /*   else if (gamepad2.circle){
                    theRobot.SpecimenDeliverLow();
                }

              */
                else if(gamepad2.triangle){
                    theRobot.SpecimenDeliverHigh();
                }
                else if(gamepad2.square){
                    theRobot.SpecimenPickupFromWall();
                }




                // Claw Control
                if (gamepad2.right_bumper) {
                    theRobot.SetClawPosition(Geronimo.CLAW_MAX_POS);
                }
                else if (gamepad2.left_bumper) {
                    theRobot.SetClawPosition(Geronimo.CLAW_MIN_POS);
                }

                // intake Hanger Control
                /*
                if (gamepad2.dpad_up) {
                    theRobot.SetHangerMaxUp();
                }
                else if (gamepad2.dpad_down)
                {
                    theRobot.SetHangerMaxDown();
                }

                 */
                if (gamepad2.right_stick_x > 0.2)
                {
                    theRobot.HangerIncrementUp();
                }
                else if (gamepad2.right_stick_x < -0.2)
                {
                    theRobot.HangerDecrementDown();
                }

                // intake Star Control
                if (gamepad2.right_trigger > 0.2)
                {
                    theRobot.SetIntakeStarPower(1);
                }
                else if (gamepad2.left_trigger > 0.2)
                {
                    theRobot.SetIntakeStarPower(-1);
                }
                else {
                    theRobot.SetIntakeStarPower(0.0);
                }

                // intake star rotator control
                /*
                if (gamepad1.dpad_up)
                {
                    theRobot.SetIntakeStarRotatorPosition(Geronimo.INTAKE_ROTATOR_MAX_POS);
                }
                else if (gamepad1.dpad_down)
                {
                    theRobot.SetIntakeStarRotatorPosition(Geronimo.INTAKE_ROTATOR_MIN_POS);
                }

                 */
                if (gamepad2.dpad_left)
                {
                    theRobot.IntakeStarRotatorDecrementDown();
                    telemetry.addLine("WORKSSSSSSS DOWNNNNNN");
                }
                else if (gamepad2.dpad_right)
                {
                    theRobot.IntakeStarRotatorIncrementUp();
                    telemetry.addLine("WORKSSSSSSS UPPPPPPPPPPPP");
                }
                else if (gamepad2.dpad_up){

                    theRobot.BasketHigh();
                    telemetry.addLine("WORKSSSSSSS");
                }
               /* else if (gamepad2.dpad_down){
                    theRobot.BasketLow();
                }

                */


                theRobot.ShowTelemetry();
                telemetry.update();
            } // end while (opModeIsActive())

        }
}
