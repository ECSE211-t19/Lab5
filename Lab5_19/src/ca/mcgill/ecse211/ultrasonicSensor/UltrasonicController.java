package ca.mcgill.ecse211.ultrasonicSensor;

public interface UltrasonicController {

  public void processUSData(int distance);

  public int readUSDistance();
}
