package org.usfirst.frc.team1014.robot;

import org.usfirst.frc.team1014.robot.commands.Intake;
import org.usfirst.frc.team1014.robot.commands.Outtake;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	
	public static Joystick xboxController0 = new Joystick(0);
	public static XboxController xboxController1 = new XboxController(1);

	// xbox left back button
	Button button5 = new JoystickButton(xboxController1, 5);
	// xbox right back button
	Button button6 = new JoystickButton(xboxController1, 6);
	

	public OI () {
		// speed is set for 1 rn but idk what it should be
		double speed = 1;
		button5.whenPressed(new Intake(speed));
		button6.whenPressed(new Outtake(speed));
	}
	
	//// CREATING BUTTONS
	// One type of button is a joystick button which is any button on a
	//// joystick.
	// You create one by telling it which joystick it's on and which button
	// number it is.
	// Joystick stick = new Joystick(port);
	// Button button = new JoystickButton(stick, buttonNumber);

	// There are a few additional built in buttons you can use. Additionally,
	// by subclassing Button you can create custom triggers and bind those to
	// commands the same as any other Button.

	//// TRIGGERING COMMANDS WITH BUTTONS
	// Once you have a button, it's trivial to bind it to a button in one of
	// three ways:

	// Start the command when the button is pressed and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenPressed(new ExampleCommand());

	// Run the command while the button is being held down and interrupt it once
	// the button is released.
	// button.whileHeld(new ExampleCommand());

	// Start the command when the button is released and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenReleased(new ExampleCommand());
	
}
