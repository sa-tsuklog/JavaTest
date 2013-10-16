/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package javatest;

import MyLib.MyMath.Matrix;
import MyLib.MyMath.MatrixInvalidOperationException;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Random;

/**
 *
 * @author sa
 */
public class Kalman2x2_2 {

    /**
     * @param args the command line arguments
     */
    double deltaT = 0.01;
    
    Matrix xk;
    Matrix uk;
    Matrix qk;
    Matrix rk;
    Matrix fk;
    Matrix pk;
    Matrix zk;
    Matrix hk;
    
    double aclSigma = 1.0;
    double posSigma = 20.;
    double spdSigma = 2.0;
    
    double trueAclSigma = 1.0;
    double truePosSigma = 20.;
    double trueSpdSigma = 2.0;
    
    
    int length = 80000;
    double[] truePos;
    double[] trueSpd;
    double[] trueAcl;
    double[] observedPos;
    double[] observedSpd;
    double[] observedAcl;
    
    public void observe(int iterator) throws MatrixInvalidOperationException{
        double[][] ukVec = new double[2][1];
        ukVec[0][0] = observedAcl[iterator] * deltaT * deltaT / 2;
        ukVec[1][0] = observedAcl[iterator] * deltaT;
        uk = new Matrix(ukVec);
        
        double[][] zkVec = new double[1][1];
        zkVec[0][0] = observedPos[iterator];
        zk = new Matrix(zkVec);
    }
    
    public void predict() throws MatrixInvalidOperationException{
        
        xk = fk.mul(xk).add(uk);
        pk = fk.mul(pk).mul(fk.transposition()).add(qk);
    }
    public void update(int iterator) throws MatrixInvalidOperationException{
        Matrix ek;
        Matrix sk;
        Matrix kk;
        
        Matrix unit = new Matrix(2);
        
        ek = zk.sub(hk.mul(xk));
        sk = rk.add(hk.mul(pk).mul(hk.transposition()));
        
        kk = pk.mul(hk.transposition().mul(sk.inverse()));
        xk = xk.add(kk.mul(ek));
        pk = (unit.sub(kk.mul(hk))).mul(pk);
        System.out.println("------");
        System.out.println(sk);
        System.out.println(rk);
        System.out.println(pk);
        System.out.println(kk);
    }
    
    public void initialize(){
        double[][] xkVec = new double[2][1];
        xkVec[0][0] = 0;
        xkVec[1][0] = 0;
        xk = new Matrix(xkVec);

        double[][] fkVec = new double[2][2];
        fkVec[0][0] = 1;
        fkVec[0][1] = deltaT;
        fkVec[1][0] = 0;
        fkVec[1][1] = 1;
        fk=new Matrix(fkVec);

        double[][] qkVec = new double[2][2];
        qkVec[0][0] = Math.pow(deltaT, 4) / 4;
        qkVec[0][1] = Math.pow(deltaT, 3) / 2;
        qkVec[1][0] = Math.pow(deltaT, 3) / 2;
        qkVec[1][1] = Math.pow(deltaT, 2);
        qk = new Matrix(qkVec);
        qk = qk.mul(aclSigma * aclSigma);

        
        double[][] rkVec = new double[1][1];
        rkVec[0][0] = posSigma * posSigma;
        rk = new Matrix(rkVec);

        double[][] pkVec = new double[2][2];
        pkVec[0][0] = posSigma;
        pkVec[0][1] = 0;
        pkVec[1][0] = 0;
        pkVec[1][1] = spdSigma;
        pk = new Matrix(pkVec);
        
        double[][] hkVec = new double[1][2];
        hkVec[0][0] = 1;
        hkVec[0][1] = 0;
        hk=new Matrix(hkVec);
    }
    
    void initSystem(){
        Random rnd = new Random(2);
        
        truePos = new double[length];
        trueSpd = new double[length];
        trueAcl = new double[length];
        observedPos = new double[length];
        observedSpd = new double[length];
        observedAcl = new double[length];
        
        for (int i = 0; i < 100; i++) {
            trueAcl[i] = 50.0;
        }
        for (int i = 100; i < 1100; i++) {
            trueAcl[i] = -5.0;
        }
        for (int i = 1099; i < length; i++) {
            trueAcl[i] = -0.0;
        }
        
        trueSpd[0]=0.0;
        truePos[0]=0.0;
        for (int i = 1; i < length; i++) {
            trueSpd[i] = trueSpd[i-1] + deltaT*trueAcl[i];
            truePos[i] = truePos[i-1] + trueSpd[i-1] * deltaT + trueAcl[i] * deltaT * deltaT /2;
        }
        for (int i = 0; i < length; i++) {
            observedPos[i] = truePos[i] + rnd.nextGaussian()*truePosSigma;
            observedSpd[i] = trueSpd[i] + rnd.nextGaussian()*trueSpdSigma;
            observedAcl[i] = trueAcl[i] + rnd.nextGaussian()*trueAclSigma+0.1;
        }   
        
    }
    
    public static void main(String[] args) throws MatrixInvalidOperationException {
        new Kalman2x2_2();
    }

    public Kalman2x2_2() throws MatrixInvalidOperationException {
        initialize();
        initSystem();
        
        try {
            PrintWriter pw = new PrintWriter(new BufferedWriter(new FileWriter("kalmanTest.txt")));
        
            pw.println("#truepos truespeed trueaccel obserbedpos observedspeed observedaccel estimatedpos estimatedspeed");
            
            for (int i = 0; i < length; i++) {
                observe(i);
                predict();
                if(i%10 == 0){
                    update(i);
                }
                pw.println(i+" "+truePos[i] + " "+ observedPos[i]+ " "+ xk.getNum(0, 0)+" "+trueSpd[i] + " "+ observedSpd[i] + " "+xk.getNum(1, 0) + " "+trueAcl[i]  + " "+  observedAcl[i]);
            }
            
            pw.close();
        } catch (IOException iOException) {
        }
//        try {
//            PrintWriter pw = new PrintWriter(new BufferedWriter(new FileWriter("kalmanTest.txt")));
//            double spd = 0.0;
//            double p = 0.0;
//            double r = spdSigma*spdSigma;
//            
//            for (int i = 0; i < length; i++) {
//                spd += deltaT * observedAcl[i];
//                p += aclSigma*aclSigma*deltaT;
//                
//                if(i%100 == 0){
//                    System.out.println("---");
//                    System.out.println(p);
//                    System.out.println(r);
//                    
//                    spd = spd * (p/(p+r)) + observedSpd[i] * (r/(p+r));
//                    p = Math.sqrt((p*r))/2;
//                    
//                    System.out.println(p);
//                }
//                
//                pw.println(i+" "+truePos[i] + " "+ observedPos[i]+ " "+ 0+" "+trueSpd[i] + " "+ observedSpd[i] + " "+spd + " "+trueAcl[i]  + " "+  observedAcl[i]);
//            }
//            
//            
//            pw.close();
//        } catch (IOException iOException) {
//            
//        }

        
        
    }
}
