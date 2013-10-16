/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package javatest;

import MyLib.DSP.DspInvalidValueException;
import MyLib.DSP.IIRFilter;
import MyLib.DSP.Wave;
import MyLib.FFT.FFT;
import MyLib.FFT.FftInvalidValueException;
import MyLib.FFT.WindowType;
import java.util.Random;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 *
 * @author sa
 */
public class SeekerSim {
    int windowLength = 1024;
    int frequencyResolution = windowLength;
    int shiftLength=windowLength/2;
    WindowType windowType = WindowType.HAMMING;
    
    //params for target wave
    double frequency=1000;
    int secLength=8;
    int samplerate=20*1000;
    int secOffset =1;
    double duty = 0.5;
    //params for noise.
    double amplitude=1.0;
    
    abstract class Seeker{
        abstract public double[] filtering(double[] data)throws DspInvalidValueException;
    }
    
    class Bandpass extends Seeker{
        public double[] filtering(double[] data) throws DspInvalidValueException {
            double[] bandpassed = IIRFilter.bandPass(data, frequency / samplerate * 2, 1000);
            double[] allpassed = IIRFilter.allPass(bandpassed, frequency / samplerate * 2);
            double[] abs = new double[data.length];

            for (int i = 0; i < data.length; i++) {
                abs[i] = Math.sqrt(bandpassed[i] * bandpassed[i] + allpassed[i] * allpassed[i]);
            }
            return abs;
        }
    }
    class IdealCorrelation extends Seeker{
        public double[] filtering(double[] data) throws DspInvalidValueException {
            final int tau =2000;
            double[] ref = getRefWave();

            double[] tmp = new double[data.length];
            for (int i = 0; i < data.length; i++) {
                tmp[i] = ref[i] * data[i];
            }
            //double[] filtered = IIRFilter.lowPass(tmp, 0.0001);
            double[] filtered = new double[data.length];

            for (int i = 0; i < data.length; i += tau) {
                double sum = 0.0;
                for (int j = 0; j < tau; j++) {
                    sum += tmp[i + j];
                }
                double average = sum / tau;
                for (int j = 0; j < tau; j++) {
                    filtered[i + j] = average;
                }
            }


            return filtered;
        }
    }
    
    public SeekerSim() {
        try {
            FFT fft = new FFT(windowLength, frequencyResolution, shiftLength, windowType);
            //Seeker seeker = new Bandpass();
            Seeker seeker = new IdealCorrelation();
            
            double noisefloor = calcNoiseFloor(seeker);
            double signalIntensity = calcSignalIntensity(seeker);
            double response = calcResponseTime(seeker, signalIntensity);
            double[] filtered = calcFilteredWave(seeker);
            double[] stepResponse = calcStepResponse(seeker);
            
            System.out.println("noise:"+noisefloor);
            System.out.println("intensity:"+signalIntensity);
            System.out.println("s/n:"+signalIntensity/noisefloor);
            System.out.println("response:"+response);
            
            
            fft.printFrame("filtered", filtered, samplerate);
            fft.printFrame("stepResponse", stepResponse,samplerate);
            //fft.printFrame("test2", fft.cutToFrame(data, samplerate, 1000));
            
            //double[] data = getNoise();            
//            double[] data = getNoise();
//            double[] filtered = seeker.filtering(data);
//            
//            
//            fft.printFrame("noise", fft.cutToFrame(filtered, samplerate*4, (int)(1000*samplerate/frequency)));
            
        } catch (Exception ex) {
            Logger.getLogger(SeekerSim.class.getName()).log(Level.SEVERE, null, ex);
        }
    }
    
    private double calcNoiseFloor(Seeker seeker) throws DspInvalidValueException{
        double[] data = getNoise();
        double[] filtered = seeker.filtering(data);
        double sum=0.0;
        
        for (int i = 0; i < filtered.length; i++) {
            sum += filtered[i]*filtered[i];
        }
        return Math.sqrt(sum/filtered.length);
    }
    private double calcSignalIntensity(Seeker seeker)throws DspInvalidValueException{
        double[] data = getRawWave();
        double[] filtered = seeker.filtering(data);
        double sum=0;
        
        for (int i = filtered.length-samplerate; i < filtered.length; i++) {
            sum+=filtered[i];
        }
        return sum/samplerate;
    }
    private double calcResponseTime(Seeker seeker,double intensity)throws DspInvalidValueException{
        double[] data = getTargetWave(0.0);
        double[] filtered = seeker.filtering(data);
        int i;
        for (i = 0; i < filtered.length && filtered[i]<intensity*0.96; i++) {}
        
        return (i-samplerate*secOffset)/(double)samplerate;
    }
    private double[] calcFilteredWave(Seeker seeker)throws DspInvalidValueException{
        double[] data = getTargetWave(amplitude);
        double[] filtered = seeker.filtering(data);
        return filtered;
    }
    private double[] calcStepResponse(Seeker seeker)throws DspInvalidValueException{
        double[] data = getStep();
        double[] filtered = seeker.filtering(data);
        return filtered;
    }
    
    
    

    
    private double[] getTargetWave(double noiselevel){
        double[] data = getRawWave();
        //double[] data = new double[samplerate*secLength];
        Random rnd = new Random(0);
        
        for (int i = 0; i < secOffset * samplerate; i++) {
            data[i] = 0;
        }
        for (int i = 0; i < data.length; i++) {
            data[i] += noiselevel*rnd.nextGaussian();
        }
        
        
        return data;
    }
    private double[] getRefWave(){
        double[] data = getRawWave();
        double sum=0;
        double average;
//        for (int i = 0; i < data.length; i++) {
//            sum += data[i];
//        }
//        average=sum/data.length;
//        
        for (int i = 0; i < data.length; i++) {
            data[i]-=1.0;
        }
        return data;
    }
    
    
    private double[] getRawWave(){
        double[] data = new double[secLength * samplerate];
        
        for (int i = 0; i < secLength * samplerate; i++) {
            data[i] = 0.5/duty*(Wave.rectangle(frequency * i * 2 * Math.PI / samplerate,duty)+1);
//            if(i%10<5){
//                data[i] = 2.0;
//            }else{
//                data[i] = 0.0;
//            }
            
            
            //data[i] = Math.sin(frequency*i*2*Math.PI/samplerate);
        }
        
        return data;
    }
    
    private double[] getNoise(){
        double[] data = new double[secLength*samplerate];
        Random rnd = new Random(0);
        
        for (int i = 0; i < data.length; i++) {
            data[i] = rnd.nextGaussian();
        }
        return data;
    }
    private double[] getStep(){
        double[] data = new double[secLength*samplerate];
        for (int i = 0; i < samplerate*secLength; i++) {
            data[i]=1;
        }
        for (int i = 0; i < samplerate*1.21353; i++) {
            data[i]=0;
        }
        
        return data;
    }
}
