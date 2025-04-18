package frc.robot;

public final class Constants {

    public static class OperatorConstants {
      public static final int kDriverControllerPort = 0;
    }
  
    public static class DriveConstants {
      public static final int kLeftMotorPort = 1;
      public static final int kRightMotorPort = 0;
      public static final int kRearMotorPort = 2;
  
      public static final double kDriveExponetialConstant = 2.0;
      public static final double kRotExponetialConstant = 2.0;
  
      public static final double kDriveMaxXYConstant = 0.55;
      public static final double kDriveMaxRConstant = 0.2;
    
      public static final double kDriveDeadbandXYConstant = 0.05;
      public static final double kDriveDeadbandRConstant = 0.05;

      public static final double kPDriveRotPIDConstant = 0.0042;
      public static final double kIDriveRotPIDConstant = 0.000;
      public static final double kDDriveRotPIDConstant = 0.00045;
     
    }

    public static class ledConstants {
      public static final int kPort = 0;
      public static final int kLength = 36;
      
    }
}