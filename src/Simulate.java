/**
 * @author NeilHazra
 * Java code used to simulate an encoder on a wheel spinning at 3200 RPM
 * Added Random error to simulate missed counts and chance variation
 *  
 * Purpose: determine most resistant/accurate/precise way to measure velocity on a spinning wheel 
 * Methods of velocity calculation: 
 *  1) delta position/ delta time
 *  2) delta position/ constant time --> change in position will be measured periodically
 *  3) Tracking loop (similar to a Kalman Filter)
 *  4) constant position/ delta time --> in practice this will be interrupt driven: the robot will measure the time between two successive ticks
 *  									 in practice it will kill CPU and will have high frequency noise
 *  									 we will test it anyway
 */
/*  Here is the outline of a tracking loop:
 *  We estimate the position by integrating a variable we call velocity_estimated
 *  We compare this to the measured position which is the extent of our knowledge...even with the assumed error
 *  
 *  We want to minimize the position_estimated - position_measured which is our error. Feed this into our PI controller and it will try to minimize the error by giving 
 *  more accurate values of velocity_estimated. 
 *  
 *  Even though this is a round about way to get at the velocity, this is good. By integrating position we rule out high frequency error 
 *  and hope the error will cancel.
 *  See for yourself... estimated position is almost always closer to the hypothetical position_exact than measured position is
 *  and velocity_estimated is more precise and consistent (with much less noise) than delta position/ delta time 
 */
import util.PIDMain;
import util.PIDSource;

public class Simulate {
	//Constants to change
	private static final int samplingTime = 10; //How quickly to calculate velocity or update PI controller
	private static final double magnitude_error = 10; //range of randomness essentially how much error will be present
	
	
	static double position_exact = 1; //the exact position of the wheel, this is the hypothetical value that we can never calculate because of error
	static double position_measured = 0; //the value of the encoder as measured, including random error
	
	static double prevPosition = 0; //variable to store previous position when calculating change in position
	static long prevTime = 0; //variable to calculate dt
	
	static double position_estimated = 0; //Estimated position calculated by integrating position
	static double velocity_estimated = 0; //velocity estimated...output of a PI controller that tries to minimize the error between position_measured and position_estimated
	
	static double velocity = 0;
	//Gains for the PI controller kd = 0 since no derivative action
	static double kp = 0.002;
	static double ki = 0.005;
	static double kd = 0.0;

	//MAPE variables
	static double M_pos_measured = 0;
	static double M_pos_estimated= 0;
	static double M_vel_measured = 0;
	static double M_vel_estimated = 0;
	static int n = 1;
	
	//Standard Deviation Variables
	static double variance_pos_measured_error = 0;
	static double mean_pos_measured_error = 0;
	
	static double variance_pos_estimated_error = 0;
	static double mean_pos_estimated_error = 0;
	
	static double variance_vel_measured_error = 0;
	static double mean_vel_measured_error = 0;
	
	static double variance_vel_estimated_error = 0;
	static double mean_vel_estimated_error = 0;
	
	
	//Anonymous inner class that implements PIDSource and allows PI controller to read input value which is the position error
	static PIDSource position_error = new PIDSource() {
		public double getInput() {
			 return position_exact - position_estimated;//without error 
			//return position_measured - position_estimated; //with error
		}
	};
	//Create the PID object this starts the timer task hidden away in the util package
	//samplingTime is how often the PI controller will run its algorithm
	static PIDMain velocityPID = new PIDMain(position_error, 0, samplingTime, kp, ki, kd);

	public static void main(String args[]) {
		(new Simulate.ManipulatePosition()).start(); //spin the wheel...aka start the simulator
		velocityPID.setOutputLimits(-10000, 10000); //default PID outputs are -1 to 1 for motors so change that
		long startTime = System.currentTimeMillis();
		System.out.println("Starting PID...");
		while (true) {
			//long delta_t = System.currentTimeMillis() - prevTime; //calculate the change in time since we will need this for multiple calculations
			long delta_t = samplingTime; //using this lets us compare the precisions of our velocity calculations
			position_estimated += velocity_estimated * delta_t; //Integrate the estimated velocity to get the estimated position 
			velocity_estimated = velocityPID.getOutput(); //grab the most recent output of the PID controller and set it as the new velocity.
														  //if PID is working this should be closer to the measured position
			//velocity = (position_measured - prevPosition) / delta_t;					
			velocity = (position_exact - prevPosition) / delta_t;					
			
			//Print some values we want to compare
				//System.out.print(position_estimated); 
				//System.out.print("\t");
				//System.out.print(position_exact);
				//System.out.print("\t");
				//System.out.print(position_measured);
				//System.out.print("\t");
				/*
				 * velocityPID.getOutput is integrated by dt where t is measured in
				 * milliseconds. As a result velocity PID has units: ticks per ms
				 * Hardware setup returns 6 ticks per wheel revolution and desired unit is RPM 
				 * 	1000ms/sec * 60sec/min * 1 rev/6 tick = 10000 rev/min
				 * Conversion factor is therefore rev/min = 10000 tick/ms
				 */
				//System.out.print(velocityPID.getOutput() * 10000); //Velocity with Kalman/Tracking filter
				//System.out.print("\t");
				//System.out.println(velocity);//velocity with regular change in position divided by change in time
	
			if(System.currentTimeMillis()-startTime>10000){ //let the PI controller start up
				//MAPE calculations
				M_pos_estimated = (n*M_pos_estimated + Math.abs((position_exact-position_estimated)/position_exact))/(n+1); //find MAPE recursively
				M_pos_measured =  (n* M_pos_measured + Math.abs((position_exact-position_measured)/position_exact))/(n+1);
				M_vel_estimated = (n* M_vel_estimated + Math.abs((3200-10_000*velocity_estimated)/3200))/(n+1);
				M_vel_measured =  (n* M_vel_measured + Math.abs((3200-10_000*velocity)/3200))/(n+1);
				//SD calculations
				mean_pos_measured_error = (mean_pos_measured_error*n + (position_measured-position_exact))/(n+1); //find Variation recursively
				variance_pos_measured_error = (n*variance_pos_measured_error+ Math.pow((position_measured-position_exact)-mean_pos_measured_error,2))/(n+1);
				mean_pos_estimated_error = (mean_pos_estimated_error*n + (position_estimated-position_exact))/(n+1);
				variance_pos_estimated_error = (n*variance_pos_estimated_error+ Math.pow((position_estimated-position_exact)-mean_pos_estimated_error,2))/(n+1);
				mean_vel_measured_error = (n*mean_vel_measured_error + (10000*velocity-3200))/(n+1);
				variance_vel_measured_error = (n*variance_vel_measured_error+ Math.pow((10000*velocity-3200)-mean_vel_measured_error,2))/(n+1);
				mean_vel_estimated_error = (mean_vel_estimated_error*n + (10000*velocity_estimated-3200))/(n+1);
				variance_vel_estimated_error = (n*variance_vel_estimated_error + Math.pow((10000*velocity_estimated-3200)-mean_vel_estimated_error,2))/(n+1);
				
				
				System.out.print(M_pos_measured*100);
				System.out.print("\t");
				System.out.print(M_pos_estimated*100);
				System.out.print("\t");
				System.out.print(M_vel_measured*100);
				System.out.print("\t");
				System.out.println(M_vel_estimated*100);
				
				//print the standard deviations
				//System.out.print(Math.sqrt(variance_pos_measured_error));
				//System.out.print("\t");
				//System.out.print(Math.sqrt(variance_pos_estimated_error));
				//System.out.print("\t");
				//System.out.print(Math.sqrt(variance_vel_measured_error));
				//System.out.print("\t");
				//System.out.println(Math.sqrt(variance_vel_estimated_error));
				
				
				n++;
			}
			
			
			/*
			 * You will notice velocityPID output has clean mostly consistent output values while delta pos/ delta t has very large values with high noise
			 * delta pos/ delta t improves with a larger sampling time but that has its own drawbacks 
			 */
			
			//Save some variables we need next time around
			prevTime = System.currentTimeMillis(); 
			//prevPosition = position_measured;
			prevPosition = position_exact;
			try {
				Thread.sleep(samplingTime); //try not to kill the CPU and also simulate more realistic conditions
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}

	protected static class ManipulatePosition extends Thread {
		public void run() {
			while (true) {
				position_exact += 1;
				try {
					Thread.sleep(3, 125000); // 3200 rpm results in 320 ticks per second which is about a tick ever 3.125 ms
				} catch (InterruptedException e) {
					e.printStackTrace();
				}		
				double error = magnitude_error * Math.random() - magnitude_error / 2;
				position_measured = Math.round(position_exact + error); //add the error to the exact position
			}
		}
	}
}