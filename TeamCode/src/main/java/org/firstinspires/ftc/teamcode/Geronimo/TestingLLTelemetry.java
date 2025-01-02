package org.firstinspires.ftc.teamcode.Geronimo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//This teleop simply shows hpw many inches the robot needs to strafe(left or right)
//to align with a Limelight target.
public class TestingLLTelemetry extends LinearOpMode{
    private Geronimo control;

    @Override
    public void runOpMode(){
        control = new Geronimo(false, false, this);
        control.Init(hardwareMap);
        control.InitLimelight(hardwareMap);

        telemetry.addLine("Ready to check Limelight offset...");
        telemetry.update();

        while(opModeIsActive()){
            double strafeOffSet = control.GetStrafeOffsetInInches("block", 10.3);
            telemetry.addData("Limelight Strafe Offset (inches)", strafeOffSet);

            if(strafeOffSet > 0){
                telemetry.addLine("=> Robot would need to strafe Right");
            }
            else if(strafeOffSet<0){
                telemetry.addLine("=> Robto would need to strafe Left");
            }
            else{
                telemetry.addLine("=> Centered or No Target Found");
            }
            telemetry.update();
        }
    }
}
