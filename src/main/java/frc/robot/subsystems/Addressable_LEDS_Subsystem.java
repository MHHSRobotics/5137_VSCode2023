// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.HALValue;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class Addressable_LEDS_Subsystem extends SubsystemBase {
  /** Creates a new Addressabe_LEDS_Subsystem. */

  private AddressableLED leds1;
  private AddressableLEDBuffer ledBuffer1;
  private int x = 0;
  private boolean debounce = true;

  public Addressable_LEDS_Subsystem() {

    //Creates new LED on port 9
    leds1 = new AddressableLED(Constants.ledString1Port);
    ledBuffer1 = new AddressableLEDBuffer(150);
    leds1.setLength(ledBuffer1.getLength());
    leds1.start();
  }

  public void alternateColors(Color RGB1, Color RGB2){

    //Sets Odd and Even Leds to alternating colors
    for(var i = 0; i < ledBuffer1.getLength(); i++){
      if(i%2 != 0){//odd
        ledBuffer1.setLED(i, RGB2);

      }
      else{//even
        ledBuffer1.setLED(i, RGB1);
      

      } 
     
      
    }
    //Sets the LED to the buffer sequence
    leds1.setData(ledBuffer1);
    leds1.start();
    System.out.println("Team Colors Set");
  }

  public void moveForward(Color RGB1, Color RGB2) {
    for(int i = 0; i < ledBuffer1.getLength(); i++) {
      if (i%10 == x) {
        ledBuffer1.setLED(i, RGB1);
      } else if ((i+8)%10 == x) {
        ledBuffer1.setLED(i, RGB2);
      }
    }
    if (x == 10) {
      x = 0;
    } else if (debounce) {
      x++;
      debounce = false;
    } else {
      debounce = true;
    }
    leds1.setData(ledBuffer1);
  }

  public void moveBackward(Color RGB1, Color RGB2) {
    for(int i = 0; i < ledBuffer1.getLength(); i++) {
      if (i%10 == x) {
        ledBuffer1.setLED(i, RGB1);
      } else if ((i+2)%10 == x) {
        ledBuffer1.setLED(i, RGB2);
      }
    }
    if (x == 0) {
      x = 10;
    } else if (debounce) {
      x--;
      debounce = false;
    } else {
      debounce = true;
    }
    leds1.setData(ledBuffer1);
  }
  

  public void solidColor(Color RGB1){

    for(var i = 0; i < ledBuffer1.getLength(); i++){
      ledBuffer1.setLED(i, RGB1);

    }

    leds1.setData(ledBuffer1);
    leds1.start();
}

public void gradiant() {
  for (var i = 0; i < ledBuffer1.getLength(); i++){
    final var hue = (x + (i * 90 / ledBuffer1.getLength())) % 180;
    ledBuffer1.setHSV((i), hue, 255, 255);
  }

  x += 1;
  x %= 1620;

  leds1.setData(ledBuffer1);
}

public void pulsingGradiant(Color RGB1, Color RGB2, int movement) {
  for (int i = 0; i < ledBuffer1.getLength(); i++) {
    double r = 255*RGB1.red*((150-i)%151)/150 + 255*RGB2.red*((i)%151)/150;
    double g = 255*RGB1.green*((150-i)%151)/150 + 255*RGB2.green*((i)%151)/150;
    double b = 255*RGB1.blue*((150-i)%151)/150 + 255*RGB2.blue*((i)%151)/150;
    ledBuffer1.setRGB((int)(i+x)%150, (int)r, (int)g, (int)b);
  }
  
  x += movement;
  x %= 150;

  leds1.setData(ledBuffer1);
}

public void s (Color RGB1, Color RGB2, int movement){
  double r = RGB1.red;
  double g = RGB1.green;
  double b = RGB1.blue;
  for (int i = 0; i < ledBuffer1.getLength(); i++){
    r = compare(r, RGB2.red);
    g = compare(r, RGB2.green);
    b = compare(b, RGB2.blue);

    ledBuffer1.setRGB(i, (int)r, (int)g, (int)b);
  }

  leds1.setData(ledBuffer1);
}

public double compare (double old, double neww){
  if (old > neww){
    return old--;
  }
  else if (old < neww){
    return old++;
  }
  return old;
}
  
public void rainbow() {
  // For every pixel
  for (var i = 0; i < ledBuffer1.getLength(); i++) {
    // Calculate the hue - hue is easier for rainbows because the color
    // shape is a circle so only one value needs to precess
    final var hue = (x + (i * 180 / ledBuffer1.getLength())) % 180;
    // Set the value
    ledBuffer1.setHSV(i, hue, 255, 128);
  }
  // Increase by to make the rainbow "move"
  x += 3;
  // Check bounds
  x %= 180;
  leds1.setData(ledBuffer1);
}


  @Override
  public void periodic() {

  }
}
