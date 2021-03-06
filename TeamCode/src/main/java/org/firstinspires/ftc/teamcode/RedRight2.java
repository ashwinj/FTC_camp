//Run code for the red right position (1 wobble goal)

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name = "RedRight2", group = "Taus")
public class RedRight2 extends AutonomousMethods {

    @Override
    public void runOpMode() throws InterruptedException {

        //initializing robot
        initializeRobot();
        //detect number of rings
        int numberOfRings = findNumRings(bmp);
        telemetry.addData("rings", numberOfRings);
        telemetry.update();
        bmp.recycle();


        stopAndResetEncoders();
        controlIndexServo(1);
        controlBlocker(.375);

        //pick up wobble goal
        controlClawServo(.25);//close
        controlArmServo(.25);//down
        //shoot
        robot.shooter.setVelocityPIDFCoefficients(p,i,d,f);
        robot.shooter.setVelocity(powerShotPower);

        switch (numberOfRings){
            case 0:
                //code
                forward(.5, 2, 10);
                shoot(-26.0, (2195*28)/60.0);
                toAngle(145, 1);
                dropWobbleGoal();
                strafeRight(.5, 0, 3);
                toAngle(0, 1);
                backward(.5, 1, 0);
                break;
            case 1:
                //code
                forward(.5, 2, 10);//16
                shoot(-28.0, (2195*28)/60.0);
                toAngle(0, 1);
                forward(.5, 1, 26);//21
                toAngle(-15,1);
                dropWobbleGoal();
                toAngle(0,1);
                strafeRight(.5, 0, 3);
                backward(.5, 2, 21);
                toAngle(90, 1);
                setIntakePower(1);
                backward(.5, 0, 12);
                forward(.5, 0, 12);
                toAngle(0, 1);
                forward(.5, 0, 20);
                shoot(-30.0, (2195*28)/60.0);
                toAngle(0, 1);
                forward(.5, 0, 12);
                break;
            case 4:
                //code
                forward(.5, 1, 15);
                shoot(-26, (2265.5*28)/60.0);
                toAngle(90, 1);
                setIntakePower(1);
                backward(1, 0, 7, 6, 2, 0);
                forward(1, 0, 7);
                shoot(-24, (2265.5*28)/60.0);
                toAngle(90, 1);
                setIntakePower(1);
                backward(.3, 0, 8);
                backward(.05, 0, 10);
                forward(.5, 0, 18);
                toAngle(0, 1);
                forward(.5, 0, 18);
                shoot(-25, (2195*28)/60.0);
                toAngle(0,1);
                forward(.5, 1, 24);
                toAngle(135, 1);
                dropWobbleGoal();
                strafeRight(.5, 0, 6);
                toAngle(0, 1);
                backward(.5, 1, 2);
                break;
        }

    }
}
