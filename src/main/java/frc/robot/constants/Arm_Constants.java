package frc.robot.constants;

public final class Arm_Constants {
   
    //CAN ID
        public final static int armRotatePort = 8; 
    
    //Degree Values
        public final static double rotateMarginOfError = 2;
        public final static double startPosition = -53; //The start position of the arm 
        public final static double flingCoastPosition = 49; //Used to determine when to cut motors for catapult.
        public final static double flingEndPosition = 52; //Used to determine when the command for preset is finished. 
      

  


    //Motor Speeds
        //Edit all for testing
        public final static double flingSpeed = 0.6; //Speed to catapult piece .8
        public final static double reloadSpeed = 0.3; //Speed to move arm otherwise .6
        public final static double manualRotateSpeed = 0.3; //CANNOT BE TRUSTED

          //PID
        public final static double rKP = reloadSpeed/8; // 
        public final static double rKI = 0;
        public final static double rKD = 0;
 
} 