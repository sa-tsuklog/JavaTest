/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package javatest;

import MyLib.DSP.DspInvalidValueException;
import MyLib.DSP.DspStat;
import MyLib.DSP.IIRFilter;
import MyLib.FFT.FFT;
import MyLib.FFT.WindowType;
import MyLib.MyMath.Matrix;
import MyLib.MyMath.Quaternion;
import gnu.io.CommPort;
import gnu.io.CommPortIdentifier;
import gnu.io.NoSuchPortException;
import gnu.io.PortInUseException;
import gnu.io.SerialPort;
import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.PrintStream;
import static java.lang.Thread.sleep;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.Random;
import java.util.logging.Level;
import java.util.logging.Logger;



/**
 *
 * @author sa
 */
public class JavaTest {
    public static void main(String[] args){
        JavaTest jt = new JavaTest();
    }
    JavaTest(){
        double[] noise = new double[4096];
        double[] bandPassed;
        double freq=2000;
        double samplerate = 20000;
        
        Random rnd = new Random();
        for (int i = 0; i < noise.length; i++) {
            //noise[i] = rnd.nextGaussian();
            noise[i] = Math.sin(2*Math.PI*i*freq/20000);
        }
        
        System.out.println("raw noise = "+DspStat.rms(noise));
        try {
            bandPassed = IIRFilter.bandPass(noise, freq/(samplerate/2) , 100);
            
            System.out.println("band passed = "+DspStat.rms(bandPassed));
            
            FFT fft = new FFT(4096, 4096, 4096, WindowType.RECT);
            
            fft.printFrame("bandPassNoise",bandPassed, samplerate);
            fft.printSpectrum("bandPassedSpectrum", fft.fft(bandPassed), false, samplerate);
            
        } catch (Exception ex) {
            Logger.getLogger(JavaTest.class.getName()).log(Level.SEVERE, null, ex);
        }
        
        
    }
    
}
