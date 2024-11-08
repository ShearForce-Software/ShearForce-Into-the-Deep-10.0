/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Test FourArmMotors_LimitSwitches", group="Test")
//@Disabled
public class TestFourArmMotors_LimitSwitches extends LinearOpMode {

    /* Declare OpMode members. */
    DcMotor leftRotater;
    DcMotor rightRotater;
    DcMotor slideLeft;
    DcMotor slideRight;
    TouchSensor touchSensorRight;
    TouchSensor touchSensorLeft;
    TouchSensor touchSensorRotator;

    public double slidePower = 0.0;


    //  private ElapsedTime     runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
  /*  static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

   */

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        // ************* Slide MOTORS ****************
        leftRotater = hardwareMap.get(DcMotorEx.class, "leftRotater");
        rightRotater = hardwareMap.get(DcMotorEx.class, "rightRotater");
        slideLeft = hardwareMap.get(DcMotorEx.class, "slideLeft");
        slideRight = hardwareMap.get(DcMotorEx.class, "slideRight");

        touchSensorRight = hardwareMap.get(TouchSensor.class, "sensor_touchRight");
        touchSensorLeft = hardwareMap.get(TouchSensor.class, "sensor_touchLeft");
        touchSensorRotator = hardwareMap.get(TouchSensor.class, "sensor_touchRotate");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftRotater.setDirection(DcMotor.Direction.REVERSE);
        rightRotater.setDirection(DcMotorSimple.Direction.FORWARD);
        slideLeft.setDirection(DcMotor.Direction.REVERSE);
        slideRight.setDirection(DcMotorSimple.Direction.FORWARD);

        leftRotater.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRotater.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //idk if this is necessary...
        leftRotater.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRotater.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", "%7d :%7d",
                leftRotater.getCurrentPosition(),
                rightRotater.getCurrentPosition(),
                slideLeft.getCurrentPosition(),
                slideRight.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        while (opModeIsActive()) {
        /*
            public void SetSlidePower(double power){
                if ((touchSensorRotator.isPressed() && power < 0))
                {
                    slidePower = 0;
                    leftRotater.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightRotater.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                else
                {
                    slidePower = power;
                }
                leftRotater.setPower(slidePower);
                rightRotater.setPower(slidePower);
            }

         */


            // Horizontal extension motors
            if (gamepad2.left_stick_y > 0.1) {
                slideLeft.setPower(gamepad2.left_stick_y);
                slideRight.setPower(gamepad2.left_stick_y);
                slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } else if (gamepad2.left_stick_y <= 0.1) {
                slideLeft.setPower(gamepad2.left_stick_y);
                slideRight.setPower(gamepad2.left_stick_y);
                slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            }else
            {
                  slideLeft.setPower(0);
                  slideRight.setPower(0);
            }


            //Rotater Motors
            // use something digital rather than analog (ie. a button click)



            if (gamepad2.right_stick_y> 0.1) {
                rightRotater.setPower(gamepad2.right_stick_y);
                leftRotater.setPower(gamepad2.right_stick_y);
                rightRotater.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftRotater.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }


                /*
                rightRotater.setTargetPosition(-1800);
                leftRotater.setTargetPosition(-1800);
                rightRotater.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftRotater.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightRotater.setPower(0.5);
                leftRotater.setPower(0.5);




              else if (gamepad2.right_stick_y <= -0.1) {
                rightRotater.setPower(gamepad2.right_stick_y);
                leftRotater.setPower(gamepad2.right_stick_y);
                rightRotater.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftRotater.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }


                /*
                rightRotater.setTargetPosition(0);
                leftRotater.setTargetPosition(0);
                rightRotater.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftRotater.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightRotater.setPower(0.5);
                leftRotater.setPower(0.5);

                 */

            else
            {
                rightRotater.setPower(0);
                leftRotater.setPower(0);
            }
            // TOUCH SENSORS
            if (touchSensorRight.isPressed()) {
                telemetry.addData("Touch Sensor Right", "Is Pressed");
                slideRight.setPower(0);
                slideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else {
                slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                telemetry.addData("Touch Sensor Right", "Is Not Pressed");
            }
            if (touchSensorLeft.isPressed()) {
                telemetry.addData("Touch Sensor Left", "Is Pressed");
                slideLeft.setPower(0);
                slideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else {
                slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                telemetry.addData("Touch Sensor Left", "Is Not Pressed");
            }

            //Rotaters Touch Sensor. Only Use ONE because of the connection.
            if (touchSensorRotator.isPressed()) {
                telemetry.addData("Touch Sensor Rotate", "Is Pressed");
                leftRotater.setPower(0);
                rightRotater.setPower(0);
                leftRotater.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightRotater.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            } else {
                leftRotater.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightRotater.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                telemetry.addData("Touch Sensor Rotater", "Is Not Pressed");
            }



            telemetry.addData("Motor Ticks slideLeft: ", slideLeft.getCurrentPosition());
            telemetry.addData("Motor Ticks slideRight: ", slideRight.getCurrentPosition());
            telemetry.addData("Motor Ticks rightRotater: ", rightRotater.getCurrentPosition());
            telemetry.addData("Motor Ticks leftRotater: ", leftRotater.getCurrentPosition());

            telemetry.addData(">", "gamepad2.right_stick_y for rotater motors");
            telemetry.addData(">", "gamepad2.left_stick_y for horizontal extension motors");


            telemetry.update();

        }
    }
}
