
package org.usfirst.frc.team5811.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team5811.robot.commands.ExampleCommand;
import org.usfirst.frc.team5811.robot.subsystems.ExampleSubsystem;

import com.kauailabs.navx.frc.AHRS;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	public static final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
	public static OI oi;

	Command autonomousCommand;
	SendableChooser<Command> chooser = new SendableChooser<>();
	
	AHRS ahrs;
    Victor frontLeftDriveMotor;
    Victor frontRightDriveMotor;
    Victor backLeftDriveMotor;
    Victor backRightDriveMotor;
	
	Joystick joystick;

    JoystickButton logitechY;
    JoystickButton logitechA;
    JoystickButton logitechX;
    JoystickButton logitechB;
    JoystickButton logitechLeftBumper;
    JoystickButton logitechRightBumper;
    
    PowerDistributionPanel power = new PowerDistributionPanel();
    
    double leftSpeed;
    double rightSpeed;

    double throttleGain;
    double turningGain;
    
    int cycleCounter;
    
    double autoSelecter;
    
    private void driveMotors(double speedLeftDM, double speedRightDM) {
    	System.out.println("Command: " + speedLeftDM);
    	frontLeftDriveMotor.set(speedLeftDM);
    	frontRightDriveMotor.set(speedRightDM);
    	backLeftDriveMotor.set(speedLeftDM);
    	backRightDriveMotor.set(speedRightDM);
    }
    
    
    private void arcadeDrive(double throttle, double turn){
    	leftSpeed = throttle * throttleGain + turn * turningGain;
    	rightSpeed = throttle * throttleGain - turn * turningGain;
    	
    	driveMotors(leftSpeed, rightSpeed);
    	
    }
    
    private void turnMacro(float degrees){
    	ahrs.reset();
    	float nowRot = (float) ahrs.getAngle();
    	double outputDirection;
    	double outputPower;
    	while(degrees+5 > nowRot && nowRot > degrees-5){
    		if(nowRot > degrees){
    			outputDirection = 1;
    		}else{
    			outputDirection = -1;
    		}
    		outputPower = outputDirection*(((nowRot-degrees)/200)+.1);
    		driveMotors(outputPower,outputPower);
    		nowRot = (float) ahrs.getAngle();
    	}
    }
	
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		oi = new OI();
		chooser.addDefault("Default Auto", new ExampleCommand());
		// chooser.addObject("My Auto", new MyAutoCommand());
		SmartDashboard.putData("Auto mode", chooser);
		
		// BUTTON MAPPING. REASON ITS HERE IS BECAUSE IT WAS WRONG AND THESE ARE
		// THE CORRECT VALUES
		// back button 9
		// left stick 11
		// right stick 12
		// start 10
		// lefttrigger 7
		// righttrigger 8
		// leftbumper 5
		// rightbumper 6
		// y 4
		// b 3
		// a 2
		// x 1
	
		
		joystick = new Joystick(0);
		logitechY = new JoystickButton(joystick, 4);
		logitechX = new JoystickButton(joystick, 1);
		logitechB = new JoystickButton(joystick, 3);
		logitechA = new JoystickButton(joystick, 2);
		
	     frontLeftDriveMotor = new Victor(2);
	       frontRightDriveMotor = new Victor(0);
	       backLeftDriveMotor = new Victor(3); 
	       backRightDriveMotor = new Victor(1);
	}

	
	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void autonomousInit() {
		autonomousCommand = chooser.getSelected();
		autoSelecter = SmartDashboard.getNumber("DB/Slider 0", 0.0);
	
		cycleCounter = 0;
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		
		cycleCounter++;
		
		
		if(autoSelecter == 0){
			if(cycleCounter > 0 && cycleCounter <= 1)
				turnMacro(90);
			}
		
	}

	@Override
	public void teleopInit() {
	
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		
		arcadeDrive((joystick.getRawAxis(2)) * 0.6, -(joystick.getRawAxis(1)) * 0.6); 
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
}
