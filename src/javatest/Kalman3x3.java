/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package javatest;


import MyLib.MyMath.Matrix;
import MyLib.MyMath.MatrixInvalidOperationException;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Random;

/**
 *
 * @author sa
 */
public class Kalman3x3 {

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
    
    double aclSigma = 0.01;
    double posSigma = 2.;
    double spdSigma = 10.0;
    double biasSigma = 0.01;
    
    
    double trueAclSigma = 0.0;
    double truePosSigma = 0.0;
    double trueSpdSigma = 0.0;
    
    
    int length = 1000;
    double[] truePos;
    double[] trueSpd;
    double[] trueAcl;
    double[] observedPos;
    double[] observedSpd;
    double[] observedAcl;
    
    public void observe(int iterator) throws MatrixInvalidOperationException{
        double[][] ukVec = new double[3][1];
        ukVec[0][0] = observedAcl[iterator] * deltaT * deltaT / 2;
        ukVec[1][0] = observedAcl[iterator] * deltaT;
        ukVec[2][0] = 0;
        uk = new Matrix(ukVec);
        
        double[][] zkVec = new double[2][1];
        zkVec[0][0] = observedPos[iterator];
        zkVec[1][0] = observedSpd[iterator];
        zk = new Matrix(zkVec);
    }
    
    public void predict() throws MatrixInvalidOperationException{
        
        xk = fk.mul(xk).add(uk);
        
        System.out.println(fk.mul(pk).mul(fk.transposition()));
        
        
        pk = fk.mul(pk).mul(fk.transposition()).add(qk);
    }
    public void update(int iterator) throws MatrixInvalidOperationException{
        Matrix ek;
        Matrix sk;
        Matrix kk;
        
        Matrix unit = new Matrix(3);
        
        ek = zk.sub(hk.mul(xk));
        sk = rk.add(hk.mul(pk).mul(hk.transposition()));
        
        kk = pk.mul(hk.transposition().mul(sk.inverse()));
        xk = xk.add(kk.mul(ek));
        pk = (unit.sub(kk.mul(hk))).mul(pk);
        
        System.out.println("tmp");
        System.out.println(sk.inverse());
    }
    
    public void initialize(){
        double[][] xkVec = new double[3][1];
        xkVec[0][0] = 0;
        xkVec[1][0] = 0;
        xkVec[1][0] = 0;
        xk = new Matrix(xkVec);

        double[][] fkVec = new double[3][3];
        fkVec[0][0] = 1;
        fkVec[0][1] = deltaT;
        fkVec[0][2] = deltaT*deltaT/2;
        fkVec[1][0] = 0;
        fkVec[1][1] = 1;
        fkVec[1][2] = deltaT;
        fkVec[2][0] = 0;
        fkVec[2][1] = 0;
        fkVec[2][2] = 1;
        fk=new Matrix(fkVec);

        double[][] qkVec = new double[3][3];
        qkVec[0][0] = aclSigma*aclSigma*Math.pow(deltaT, 4) / 4;
        qkVec[0][1] = aclSigma*aclSigma*Math.pow(deltaT, 3) / 2;
        qkVec[0][2] = 0;
        qkVec[1][0] = aclSigma*aclSigma*Math.pow(deltaT, 3) / 2;
        qkVec[1][1] = aclSigma*aclSigma*Math.pow(deltaT, 2);
        qkVec[1][2] = 0;
        qkVec[2][0] = 0;
        qkVec[2][1] = 0;
        qkVec[2][2] = biasSigma*biasSigma;
        
        qk = new Matrix(qkVec);

        
        double[][] rkVec = new double[2][2];
        rkVec[0][0] = posSigma * posSigma;
        rkVec[0][1] = 0;
        rkVec[1][0] = 0;
        rkVec[1][1] = spdSigma * spdSigma;
        rk = new Matrix(rkVec);

        double[][] pkVec = new double[3][3];
        pkVec[0][0] = posSigma;
        pkVec[0][1] = 0;
        pkVec[0][2] = 0;
        pkVec[1][0] = 0;
        pkVec[1][1] = spdSigma;
        pkVec[1][2] = 0;
        pkVec[2][0] = 0;
        pkVec[2][1] = 0;
        pkVec[2][2] = biasSigma;
        pk = new Matrix(pkVec);
        
        double[][] hkVec = new double[2][3];
        hkVec[0][0] = 1;
        hkVec[0][1] = 0;
        hkVec[0][2] = 0;
        hkVec[1][0] = 0;
        hkVec[1][1] = 1;
        hkVec[1][2] = 0;
        hk=new Matrix(hkVec);
    }
    
    void initSystem(){
        Random rnd = new Random(4);
        
        truePos = new double[length];
        trueSpd = new double[length];
        trueAcl = new double[length];
        observedPos = new double[length];
        observedSpd = new double[length];
        observedAcl = new double[length];
        
        for (int i = 0; i < 100; i++) {
            trueAcl[i] = 1.0;
        }
        for (int i = 100; i < 200; i++) {
            trueAcl[i] = -1.0;
        }
        for (int i = 1100; i < length; i++) {
            trueAcl[i] = -0.0;
        }
        
        trueSpd[0]=0.0;
        truePos[0]=0.0;
        for (int i = 1; i < length; i++) {
            trueSpd[i] = trueSpd[i-1] + deltaT*trueAcl[i];
            truePos[i] = truePos[i-1] + trueSpd[i-1] * deltaT + trueAcl[i] * deltaT * deltaT /2;
        }
        
        try {
            BufferedReader br = new BufferedReader(new FileReader("acl.txt"));
        
            for (int i = 0; i < length; i++) {
//                observedPos[i] = truePos[i] + rnd.nextGaussian()*truePosSigma;
//                observedSpd[i] = trueSpd[i] + rnd.nextGaussian()*trueSpdSigma;
//                observedAcl[i] = trueAcl[i] + rnd.nextGaussian()*trueAclSigma;
                String line = br.readLine();
                System.out.println(line);
                String[] lines = line.split(",");
                observedAcl[i] = Double.parseDouble(lines[0])*9.8;
                observedSpd[i] = Double.parseDouble(lines[1]);
                observedPos[i] = Double.parseDouble(lines[2]);
            }   
        
            br.close();
        } catch (IOException iOException) {
        }
        
    }
    
    public static void main(String[] args) throws MatrixInvalidOperationException {
        new Kalman3x3();
    }

    public Kalman3x3() throws MatrixInvalidOperationException {
        initialize();
        initSystem();
        
        try {
            PrintWriter pw = new PrintWriter(new BufferedWriter(new FileWriter("kalmanTest.txt")));
        
            pw.println("#truepos truespeed trueaccel obserbedpos observedspeed observedaccel estimatedpos estimatedspeed");
            
            System.out.println(pk);
            for (int i = 0; i < length; i++) {
                observe(i);
                predict();
                if(i%10 == 0){
                    update(i);
                }
                System.out.println(pk);
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
