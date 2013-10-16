/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package javatest;

import java.awt.AWTException;
import java.awt.Robot;
import java.awt.event.InputEvent;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 *
 * @author sa
 */
public class AutoClicker {
    static final int MS_SLEEPTIME = 5;
    static final int SEC_CLICKTIME = 77;
    /**
     * @param args the command line arguments
     */
    public static void main(String[] args) {
        AutoClicker autoClicker = new AutoClicker();
    }
    public AutoClicker(){
        
        
        try {
            Robot robot = new Robot();
            
            robot.mouseMove(1100, 900);
            
            for (int i = 0; i < 1000/MS_SLEEPTIME*SEC_CLICKTIME; i++) {
                robot.mousePress(InputEvent.BUTTON1_MASK);
                robot.mouseRelease(InputEvent.BUTTON1_MASK);
                Thread.sleep(MS_SLEEPTIME);
                
                int secTime = i/(1000/MS_SLEEPTIME);
                if(i%(1000/MS_SLEEPTIME) == 0){
                }                    System.out.println(SEC_CLICKTIME-secTime);

            }
            
            
            
        } catch (Exception ex) {
            Logger.getLogger(AutoClicker.class.getName()).log(Level.SEVERE, null, ex);
        }
    }
}
