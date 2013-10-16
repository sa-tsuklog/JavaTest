/*
 *  ins/kalman filter.
 *  バイアス推定無しでうごいたところのバックアップ。
 * クラス分けをちゃんとやってない。
 */
package javatest;

import gnu.io.*;
import gnu.io.UnsupportedCommOperationException;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Random;
import java.util.TooManyListenersException;
import java.util.logging.Level;
import java.util.logging.Logger;
import MyLib.MyMath.*;
import java.text.NumberFormat;
import org.omg.CORBA.MARSHAL;


public class JavaTestMain{

    //final Quaternion earthRate=new Quaternion(0, 0, 0, 2*Math.PI/24/60/60);//[rad/sec]
    final Quaternion earthRate=new Quaternion(0, 0, 0, 0);//[rad/sec]
    final Quaternion gravity=new Quaternion(0, 0, 0, 9.8);//[m/s]
    //final Quaternion gravity=new Quaternion(0, 0, 0, 0);//[m/s]
    final double earthRadius=6378137;
    final double decentering=0.0818191908426;

    public static void main(String[] args){
        JavaTestMain jtm=new JavaTestMain();

    }
    JavaTestMain(){
        //TODO INS part may have error only in x axis.
        {
            try {
                PrintWriter pw = new PrintWriter(new BufferedWriter(new FileWriter("test.txt")));

                Quaternion P=new Quaternion(0,1,0,0);
                Quaternion tmp;
                double norm;

                Quaternion iToE=new Quaternion(1, 0, 0, 0);

                double sTimeStep=0.01;

                double r=100.;
                double v=2*r*Math.PI/10;

                Quaternion velocity=new Quaternion(0, v, 0, 0);
                Quaternion position=angleToPosition(-100.0/earthRadius, 0, 0);
                Quaternion attitude=new Quaternion(1,0,0,0);
                double height=0;

                Quaternion gpsPos;
                Quaternion gpsVel;
                double gpsHeight=0;
                Quaternion truePos;
                Quaternion trueVel;

                Matrix insState;
                Matrix gpsState;

                Quaternion accelBase= new Quaternion(0,0,v*v/r,-9.8);
                //Quaternion accelBase= new Quaternion(0,0,v*v/r,0);
                Quaternion gyroBase = new Quaternion(0,0,0,2*Math.PI/10);
                Quaternion accelOut=accelBase;
                Quaternion gyroOut=gyroBase;

                Quaternion eToG=updateEtoG(position);
                Quaternion gToN=updateGtoN(position);
                Quaternion eToN=eToG.mul(gToN);

                Quaternion tmpVelocity;
                Quaternion tmpPosition;
                Quaternion tmpAttitude;
                double tmpHeight;

                Matrix sysErrorMatrixPhi;
                Matrix ctlErrorMatrixGamma;
                Matrix p=initializeP(r,v);
                Matrix ctlErrorCovQ;
                Matrix sysErrorCovS;
                Matrix kalmanGainK;
                Matrix estimatedErrorDx;


                Random rnd = new Random(System.nanoTime());
                Quaternion noise;

                for (int i = 0; i < 10000; i++) {
                    noise=getNoise(rnd, 0.1*v*v/r);
                    accelOut = accelBase.add(noise);
                    noise=getNoise(rnd,0.1*2*Math.PI);
                    gyroOut  = gyroBase.add(noise);

                    tmpVelocity = updateVelocity(accelOut, position, velocity, attitude, height, eToG, gToN, sTimeStep);
                    tmpPosition = updatePosition(position, velocity, height, eToG, gToN, sTimeStep);
                    tmpAttitude = updateAttitude(gyroOut, position, velocity, height, attitude, eToG, gToN, sTimeStep);
                    tmpHeight   = updateHeight(height, velocity, sTimeStep);

                    velocity=tmpVelocity;
                    position=tmpPosition;
                    attitude=tmpAttitude;
                    height  =tmpHeight;

                    eToG     = updateEtoG(position);
                    gToN     = updateGtoN(position);
                    eToN     = eToG.mul(gToN);

                    tmp=attitude.con().mul(P).mul(attitude);

                    sysErrorMatrixPhi=getSysErrorMatrixPhi(accelOut, position, velocity, height, attitude, eToN, sTimeStep);
                    ctlErrorMatrixGamma=getCtlErrorMatrixGamma(attitude, sTimeStep);
                    ctlErrorCovQ=getCtlErrorCovQ(v,r);
                    sysErrorCovS=getSysErrorCovS();
                    p=timeUpdateP(p, ctlErrorCovQ, sysErrorMatrixPhi, ctlErrorMatrixGamma, sysErrorCovS);

                    if(i%10==0){
                        gpsPos=getSimGpsPos(r,i,rnd,0.1);
                        gpsVel=getSimGpsVel(v, i, rnd, 0.001);
                        gpsHeight=0;
                        kalmanGainK=getKalmanGainK(p, getGpsErrorCovR(v,r), getH_delta1(position));

                        insState=insWrapParamsToState(velocity, position, height, attitude);
                        gpsState=gpsWrapParamsToState(gpsVel, gpsPos, gpsHeight);
                        estimatedErrorDx=getEstimatedError(insState, gpsState, kalmanGainK, getH_delta2());

                        insState=measurementUpdateX(velocity, position, height, attitude, estimatedErrorDx);

                        p=measurementUpdateP(p, kalmanGainK, getH_delta1(position));

                        velocity=insStateToVelocity(insState);
                        position=insStateToPosition(insState);
                        height=insStateToHeight(insState);
                        attitude=insStateToAttitude(insState);
                    }else{
                        gpsPos=new Quaternion(Double.NaN, Double.NaN, Double.NaN, Double.NaN);
                        gpsVel=new Quaternion(Double.NaN, Double.NaN, Double.NaN, Double.NaN);
                    }
                    truePos=getSimTruePos(r,i);
                    trueVel=getSimTrueVel(v, i);

                    //plot position
                    pw.println(i*sTimeStep+"\t"+positionToLambda(position)*earthRadius+"\t"+positionToPhi(position)*earthRadius+"\t"
                            +positionToAlpha(position)*earthRadius+"\t"+height+"\t"
                            +positionToLambda(gpsPos)*earthRadius+"\t"+positionToPhi(gpsPos)*earthRadius+"\t"
                            +positionToLambda(truePos)*earthRadius+"\t"+positionToPhi(truePos)*earthRadius);

//                    //plot velocity
//                    pw.println(i*sTimeStep+"\t"+velocity.getX()+"\t"+velocity.getY()+"\t"
//                            +velocity.getZ()+"\t"+0+"\t"
//                            +gpsVel.getX()+"\t"+gpsVel.getY()+"\t"
//                            +trueVel.getX()+"\t"+trueVel.getY());


                }

                pw.close();
            } catch (IOException iOException) {

            }
        }
    }
    private class SimGps{
        Random rnd;
        SimGps(){
            rnd=new Random(System.nanoTime());
        }
        public Quaternion getSimGpsPos(double r,int i,Random rnd,double mStdDev){
            double xpos,ypos;
            xpos=-r*(Math.cos(2*Math.PI*i/1000)+rnd.nextGaussian()*mStdDev);
            ypos= r*(Math.sin(2*Math.PI*i/1000)+rnd.nextGaussian()*mStdDev);

            return angleToPosition(xpos/earthRadius, ypos/earthRadius, 0);
        }
        public Quaternion getSimGpsVel(double v,int i,Random rnd, double mStdDev){
            double xvel,yvel;
            xvel= v*(Math.cos(2*Math.PI*i/1000)+rnd.nextGaussian()*mStdDev);
            yvel= v*(Math.sin(2*Math.PI*i/1000)+rnd.nextGaussian()*mStdDev);

            return new Quaternion(0, xvel, yvel, 0);
        }
    }
    private class 
    private Quaternion getSimTruePos(double r,int i){
        double xpos,ypos;
        xpos=-r*Math.cos(2*Math.PI*i/1000);
        ypos=r*Math.sin(2*Math.PI*i/1000);

        return angleToPosition(xpos/earthRadius, ypos/earthRadius, 0);
    }
    private Quaternion getSimTrueVel(double v,int i){
        double xvel,yvel;
        xvel= v*Math.cos(2*Math.PI*i/1000);
        yvel= v*Math.sin(2*Math.PI*i/1000);

        return new Quaternion(0, xvel, yvel, 0);
    }
    private Quaternion getNoise(Random rnd,double stdDev){
        double x=(rnd.nextDouble()-0.5)*stdDev;
        double y=(rnd.nextDouble()-0.5)*stdDev;
        double z=(rnd.nextDouble()-0.5)*stdDev;

        return new Quaternion(0, x, y, z);
    }
    private Matrix getGpsErrorCovR(double v,double r){
        //gps error. R.
        Quaternion posError = angleToPosition(0.1*r, 0.1*r, 0);

        double[][] nums=new double[8][8];
        nums[0][0]=0.001*v;//velocity x error
        nums[1][1]=0.001*v;//velocity y error
        nums[2][2]=0.001*v;//velocity z error
        nums[3][3]=0.1;//position w error
        nums[4][4]=0.1;//position x error
        nums[5][5]=0.1;//position y error
        nums[6][6]=0.1;//position z error
        nums[7][7]=0.1*r;//height error

        return new Matrix(nums);
    }
    private Matrix getSysErrorCovS(){
        //system error. S.
        double[][] nums=new double[10][10];
        nums[0][0]=0.00001;//velocity x error
        nums[1][1]=0.00001;//velocity y error
        nums[2][2]=0.00001;//velocity z error
        nums[3][3]=0.00001;//position x error
        nums[4][4]=0.00001;//position y error
        nums[5][5]=0.00001;//position z error
        nums[6][6]=0.00001;//height error
        nums[7][7]=0.00001;//attitude x error
        nums[8][8]=0.00001;//attitude y error
        nums[9][9]=0.00001;//attitude z error

        return new Matrix(nums);
    }
    private Matrix measurementUpdateX(Quaternion velocity, Quaternion position,double height,Quaternion attitude, Matrix estimatedError){
        double[][] nums=new double[12][1];

        //velocity
        nums[0][0]=velocity.getX()-estimatedError.getNum(0, 0);
        nums[1][0]=velocity.getY()-estimatedError.getNum(1, 0);
        nums[2][0]=velocity.getZ()-estimatedError.getNum(2, 0);

        //position
        Quaternion posError=new Quaternion(1, estimatedError.getNum(3, 0), estimatedError.getNum(4, 0), estimatedError.getNum(5, 0));
        Quaternion q1=(posError.con()).mul(position);

        nums[3][0]=q1.getW();
        nums[4][0]=q1.getX();
        nums[5][0]=q1.getY();
        nums[6][0]=q1.getZ();

        //height
        nums[7][0]=height-estimatedError.getNum(6, 0);

        //attitude.
        Quaternion attError=new Quaternion(1,estimatedError.getNum(7, 0),estimatedError.getNum(8, 0),estimatedError.getNum(9, 0));
        Quaternion q2=(attError.con()).mul(attitude);

        nums[8][0]=q2.getW();
        nums[9][0]=q2.getX();
        nums[10][0]=q2.getY();
        nums[11][0]=q2.getZ();

        return new Matrix(nums);
    }
    private Quaternion insStateToVelocity(Matrix state){
        return new Quaternion(0, state.getNum(0, 0), state.getNum(1, 0), state.getNum(2, 0));
    }
    private Quaternion insStateToPosition(Matrix state){
        return new Quaternion(state.getNum(3, 0), state.getNum(4, 0) , state.getNum(5, 0), state.getNum(6, 0));
    }
    private double insStateToHeight(Matrix state){
        return state.getNum(7, 0);
    }
    private Quaternion insStateToAttitude(Matrix state){
        return new Quaternion(state.getNum(8, 0), state.getNum(9, 0), state.getNum(10, 0), state.getNum(11, 0));
    }
    private Matrix insWrapParamsToState(Quaternion velocity,Quaternion position, double height, Quaternion attitude){
        double[][] nums=new double[12][1];
        nums[0][0]=velocity.getX();
        nums[1][0]=velocity.getY();
        nums[2][0]=velocity.getZ();
        nums[3][0]=position.getW();
        nums[4][0]=position.getX();
        nums[5][0]=position.getY();
        nums[6][0]=position.getZ();
        nums[7][0]=height;
        nums[8][0]=attitude.getW();
        nums[9][0]=attitude.getX();
        nums[10][0]=attitude.getY();
        nums[11][0]=attitude.getZ();

        return new Matrix(nums);
    }
    private Matrix gpsWrapParamsToState(Quaternion gpsVelocity,Quaternion gpsPosition,double gpsHeight){
        double[][] nums=new double[8][1];
        nums[0][0]=gpsVelocity.getX();
        nums[1][0]=gpsVelocity.getY();
        nums[2][0]=gpsVelocity.getZ();
        nums[3][0]=gpsPosition.getW();
        nums[4][0]=gpsPosition.getX();
        nums[5][0]=gpsPosition.getY();
        nums[6][0]=gpsPosition.getZ();
        nums[7][0]=gpsHeight;

        return new Matrix(nums);
    }
    private Matrix initializeP(double r,double v){
        double[][] nums=new double[10][10];
        nums[0][0]=v*0.5;//velocity x error
        nums[1][1]=v*0.5;//velocity y error
        nums[2][2]=v*0.5;//velocity z error
        nums[3][3]=r*0.5;//position x error
        nums[4][4]=r*0.5;//position y error
        nums[5][5]=r*0.5;//position z error
        nums[6][6]=r*0.5;//height error
        nums[7][7]=2*Math.PI*0.1;//attitude x error
        nums[8][8]=2*Math.PI*0.1;//attitude y error
        nums[9][9]=2*Math.PI*0.1;//attitude z error

        return new Matrix(nums);
    }
    private Matrix measurementUpdateP(Matrix stateCovP,Matrix kalmanGain,Matrix h_delta1){
        Matrix unitMatrix=new Matrix(10);
        try {
            return (unitMatrix.sub(kalmanGain.mul(h_delta1))).mul(stateCovP);
        } catch (MatrixInvalidOperationException ex) {
            Logger.getLogger(JavaTestMain.class.getName()).log(Level.SEVERE, null, ex);
            return null;
        }
    }
    private Matrix getEstimatedError(Matrix insState, Matrix gpsState ,Matrix kalmanGain, Matrix h_delta2){
        try {
            //returns delta x hat _
            return kalmanGain.mul(gpsState.sub(h_delta2.mul(insState)));
        } catch (MatrixInvalidOperationException ex) {
            Logger.getLogger(JavaTestMain.class.getName()).log(Level.SEVERE, null, ex);
            return null;
        }
    }
    private Matrix getCtlErrorCovQ(double r,double v){
        //control error. Q.
        double[][] nums=new double[9][9];
        nums[0][0]=0.1*v*v/r;//accel x error;
        nums[1][1]=0.1*v*v/r;//accel y error;
        nums[2][2]=0.1*v*v/r;//accel z error;
        nums[3][3]=0.1*2*Math.PI;//gyro x error;
        nums[4][4]=0.1*2*Math.PI;//gyro y error;
        nums[5][5]=0.1*2*Math.PI;//gyro z error;
        nums[6][6]=0.1*9.8;//gravity x error;
        nums[7][7]=0.1*9.8;//gravity y error;
        nums[8][8]=0.1*9.8;//gravity z error;

        return new Matrix(nums);
    }
    private Matrix getKalmanGainK(Matrix p,Matrix gpsErrorCov,Matrix h_delta1){
        try {
            Matrix m2 = (h_delta1. mul(p). mul(h_delta1.transposition())) .add (gpsErrorCov);
            return p.mul(h_delta1.transposition()).mul(m2.inverse());
        } catch (MatrixInvalidOperationException ex) {
            Logger.getLogger(JavaTestMain.class.getName()).log(Level.SEVERE, null, ex);
            return null;
        }

    }
    private Matrix timeUpdateP(Matrix p,Matrix q,Matrix sysError,Matrix ctlError,Matrix s){
        //sysError=Phi
        //ctlError=Gamma
        try {
            Matrix m1 = sysError.mul(p).mul(sysError.transposition());
            Matrix m2 = ctlError.mul(q).mul(ctlError.transposition());
            return m1.add(m2).add(s);
        } catch (MatrixInvalidOperationException ex) {
            Logger.getLogger(JavaTestMain.class.getName()).log(Level.SEVERE, null, ex);
            return null;
        }
    }
    private Matrix getSysErrorMatrixPhi(Quaternion accelOut,Quaternion position, Quaternion velocity, double height,Quaternion attitude, Quaternion eToN,double sTimeStep){
        //returns Phi.
        Matrix[][] matrixes=new Matrix[4][4];

        matrixes[0][0]=getErrorA_00(position, velocity, height, eToN);
        matrixes[0][1]=getErrorA_01(position, velocity, height);
        matrixes[0][2]=getErrorA_02(position, velocity, height);
        matrixes[0][3]=getErrorA_03(accelOut, attitude);//TODO check accelOut part.
        matrixes[1][0]=getErrorA_10(position, height);
        matrixes[1][1]=getErrorA_11();
        matrixes[1][2]=getErrorA_12(position, velocity, height);
        matrixes[1][3]=getErrorA_13();
        matrixes[2][0]=getErrorA_20();
        matrixes[2][1]=getErrorA_21();
        matrixes[2][2]=getErrorA_22();
        matrixes[2][3]=getErrorA_23();
        matrixes[3][0]=getErrorA_30(height);
        matrixes[3][1]=getErrorA_31(position, height);
        matrixes[3][2]=getErrorA_32(velocity, height);
        matrixes[3][3]=getErrorA_33(position, velocity, height, eToN);

        try {
            Matrix m1=new Matrix(matrixes);
            Matrix m2=new Matrix(m1.getColumnNum());
            return (m1.mul(sTimeStep)).add(m2);

        } catch (MatrixInvalidOperationException ex) {
            Logger.getLogger(JavaTestMain.class.getName()).log(Level.SEVERE, null, ex);
            return null;
        }
    }
    private Matrix getCtlErrorMatrixGamma(Quaternion attitude,double sTimeStep){
        //returns Gamma

        Matrix[][] matrixes=new Matrix[4][3];

        matrixes[0][0]=getErrorB_00(attitude);
        matrixes[0][1]=getErrorB_01();
        matrixes[0][2]=getErrorB_02();
        matrixes[1][0]=getErrorB_10();
        matrixes[1][1]=getErrorB_11();
        matrixes[1][2]=getErrorB_12();
        matrixes[2][0]=getErrorB_20();
        matrixes[2][1]=getErrorB_21();
        matrixes[2][2]=getErrorB_22();
        matrixes[3][0]=getErrorB_30();
        matrixes[3][1]=getErrorB_31(attitude);
        matrixes[3][2]=getErrorB_32();
        try {
            return (new Matrix(matrixes)).mul(sTimeStep);
        } catch (MatrixInvalidOperationException ex) {
            Logger.getLogger(JavaTestMain.class.getName()).log(Level.SEVERE, null, ex);
            return null;
        }
    }
    private Matrix getH_delta1(Quaternion position){
        //returns H_delta1
        Matrix[][] matrixes=new Matrix[3][4];

        matrixes[0][0]=new Matrix(3);
        matrixes[0][1]=new Matrix(3, 3, 0);
        matrixes[0][2]=new Matrix(3, 1, 0);
        matrixes[0][3]=new Matrix(3, 3, 0);
        matrixes[1][0]=new Matrix(4, 3, 0);
        matrixes[1][1]=getPositionErrorObserveMatrix(position);
        matrixes[1][2]=new Matrix(4, 1, 0);
        matrixes[1][3]=new Matrix(4, 3, 0);
        matrixes[2][0]=new Matrix(1, 3, 0);
        matrixes[2][1]=new Matrix(1, 3, 0);
        matrixes[2][2]=new Matrix(1);
        matrixes[2][3]=new Matrix(1, 3, 0);

        Matrix m1;
        try {
            m1 = new Matrix(matrixes);

            return m1.mul(-1);

        } catch (MatrixInvalidOperationException ex) {
            Logger.getLogger(JavaTestMain.class.getName()).log(Level.SEVERE, null, ex);
            return null;

        }
    }
    private Matrix getH_delta2(){
        //returns H_delta2
        Matrix[][] matrixes=new Matrix[1][2];
        matrixes[0][0]=new Matrix(8);
        matrixes[0][1]=new Matrix(8, 4, 0);
        try {
            return new Matrix(matrixes);
        } catch (MatrixInvalidOperationException ex) {
            Logger.getLogger(JavaTestMain.class.getName()).log(Level.SEVERE, null, ex);
            return null;
        }
    }
    private Matrix getPositionErrorObserveMatrix(Quaternion position){
        double[][] nums=new double[4][3];
        nums[0][0]=-position.getX();
        nums[0][1]=-position.getY();
        nums[0][2]=-position.getZ();
        nums[1][0]= position.getW();
        nums[1][1]= position.getZ();
        nums[1][2]=-position.getY();
        nums[2][0]=-position.getZ();
        nums[2][1]= position.getW();
        nums[2][2]= position.getX();
        nums[3][0]= position.getY();
        nums[3][1]=-position.getX();
        nums[3][2]= position.getW();

        return new Matrix(nums);
    }
    private Matrix getErrorA_00(Quaternion position, Quaternion velocity, double height,Quaternion eToN){
        try {
            Vector3d w_ei_n = ((eToN.con()).mul(earthRate).mul(eToN)).getVector3d();
            Vector3d w_ne_n = getW_ne_n(position, velocity, height);
            Vector3d vectColioris = w_ei_n.mul(2.0).add(w_ne_n);

            double r_e=earthRadius;

            double[][] nums1=new double[3][3];
            nums1[0][0]=0;
            nums1[0][1]=-vectColioris.getNum(2);
            nums1[0][2]= vectColioris.getNum(1);
            nums1[1][0]= vectColioris.getNum(2);
            nums1[1][1]= 0;
            nums1[1][2]=-vectColioris.getNum(0);
            nums1[2][0]=-vectColioris.getNum(1);
            nums1[2][1]= vectColioris.getNum(0);
            nums1[2][2]= 0;

            double[][] nums2=new double[3][3];
            nums2[0][0]=velocity.getZ();
            nums2[0][1]=0;
            nums2[0][2]=0;
            nums2[1][0]=0;
            nums2[1][1]=velocity.getZ();
            nums2[1][2]=0;
            nums2[2][0]=-velocity.getX();
            nums2[2][1]=-velocity.getY();
            nums2[2][2]=0;

            Matrix m1=new Matrix(nums1);
            Matrix m2=new Matrix(nums2);

            return (m1.mul(-1)).add(m2.mul(1.0/(r_e+height)));

        } catch (MatrixInvalidOperationException ex) {
            Logger.getLogger(JavaTestMain.class.getName()).log(Level.SEVERE, null, ex);
            return null;
        }
    }
    private Matrix getErrorA_01(Quaternion position, Quaternion velocity, double height){
        try {
            Matrix dcmPos = position.getDCM();
            double[][] nums1 = new double[3][3];
            nums1[0][0] = dcmPos.getNum(2, 1) * velocity.getY() - dcmPos.getNum(1, 1) * velocity.getZ();
            nums1[0][1] = dcmPos.getNum(1, 0) * velocity.getZ() - dcmPos.getNum(2, 0) * velocity.getY();
            nums1[0][2] = 0;
            nums1[1][0] = dcmPos.getNum(0, 1) * velocity.getZ() - dcmPos.getNum(2, 1) * velocity.getX();
            nums1[1][1] = dcmPos.getNum(2, 0) * velocity.getX() - dcmPos.getNum(0, 0) * velocity.getZ();
            nums1[1][2] = 0;
            nums1[2][0] = dcmPos.getNum(1, 1) * velocity.getX() - dcmPos.getNum(0, 1) * velocity.getY();
            nums1[2][1] = dcmPos.getNum(0, 0) * velocity.getY() - dcmPos.getNum(1, 0) * velocity.getX();
            nums1[2][2] = 0;
            Matrix m1 = new Matrix(nums1);
            double[][] nums2 = new double[3][3];
            nums2[0][0] = dcmPos.getNum(0, 0) * dcmPos.getNum(1, 0) + dcmPos.getNum(0, 1) * dcmPos.getNum(1, 1) + dcmPos.getNum(0, 2) * dcmPos.getNum(2, 1);
            nums2[0][1] = -dcmPos.getNum(0, 2) * dcmPos.getNum(2, 0) - dcmPos.getNum(0, 0) * dcmPos.getNum(0, 0) - dcmPos.getNum(0, 1) * dcmPos.getNum(0, 1);
            nums2[0][2] = -dcmPos.getNum(0, 0) * dcmPos.getNum(2, 1) + dcmPos.getNum(0, 1) * dcmPos.getNum(2, 0);
            nums2[1][0] = dcmPos.getNum(1, 2) * dcmPos.getNum(2, 1) + dcmPos.getNum(1, 0) * dcmPos.getNum(1, 0) + dcmPos.getNum(1, 1) * dcmPos.getNum(1, 1);
            nums2[1][1] = -dcmPos.getNum(1, 2) * dcmPos.getNum(2, 0) - dcmPos.getNum(1, 0) * dcmPos.getNum(0, 0) - dcmPos.getNum(1, 1) * dcmPos.getNum(0, 1);
            nums2[1][2] = -dcmPos.getNum(1, 0) * dcmPos.getNum(2, 1) + dcmPos.getNum(1, 1) * dcmPos.getNum(2, 0);
            nums2[2][0] = dcmPos.getNum(2, 2) * dcmPos.getNum(2, 1) + dcmPos.getNum(2, 0) * dcmPos.getNum(1, 0) + dcmPos.getNum(2, 1) * dcmPos.getNum(1, 1);
            nums2[2][1] = -dcmPos.getNum(2, 2) * dcmPos.getNum(2, 0) - dcmPos.getNum(2, 0) * dcmPos.getNum(0, 0) - dcmPos.getNum(2, 1) * dcmPos.getNum(0, 1);
            nums2[2][2] = -dcmPos.getNum(2, 0) * dcmPos.getNum(2, 1) + dcmPos.getNum(2, 1) * dcmPos.getNum(2, 0);
            Matrix m2 = new Matrix(nums2);
            return m1.mul(4 * earthRate.getZ()).add(m2.mul(2 * earthRate.getZ() * earthRate.getZ() * (earthRadius + height)));

        } catch (MatrixInvalidOperationException ex) {
            Logger.getLogger(JavaTestMain.class.getName()).log(Level.SEVERE, null, ex);
            return null;
        }
    }
    private Matrix getErrorA_02(Quaternion position, Quaternion velocity, double height){
        try {
            double[][] nums1 = new double[3][1];
            nums1[0][0] = velocity.getX() * velocity.getZ();
            nums1[1][0] = velocity.getY() * velocity.getZ();
            nums1[2][0] = -velocity.getX() * velocity.getX() - velocity.getY() * velocity.getY();
            Matrix m1 = new Matrix(nums1);
            double r_e = earthRadius;
            Matrix dcmPos = position.getDCM();
            double[][] nums2 = new double[3][1];
            nums2[0][0] = dcmPos.getNum(0, 0) * dcmPos.getNum(2, 0) - dcmPos.getNum(0, 1) * dcmPos.getNum(2, 1);
            nums2[1][0] = dcmPos.getNum(1, 0) * dcmPos.getNum(2, 0) - dcmPos.getNum(1, 1) * dcmPos.getNum(2, 1);
            nums2[2][0] = dcmPos.getNum(2, 0) * dcmPos.getNum(2, 0) - dcmPos.getNum(2, 1) * dcmPos.getNum(2, 1);
            Matrix m2 = new Matrix(nums2);
            double dEarthRate = earthRate.getZ();
            return m1.mul(-1.0 / ((r_e + height) * (r_e + height))).sub(m2.mul(dEarthRate * dEarthRate));

        } catch (MatrixInvalidOperationException ex) {
            Logger.getLogger(JavaTestMain.class.getName()).log(Level.SEVERE, null, ex);
            return null;
        }
    }
    private Matrix getErrorA_03(Quaternion accelOut,Quaternion attitude){
        double[][] nums = new double[3][3];

        Matrix dcmAtt = attitude.getDCM();

        nums[0][0]=0;
        nums[0][1]= -dcmAtt.getNum(0, 2)*accelOut.getX() - dcmAtt.getNum(1, 2)*accelOut.getY() - dcmAtt.getNum(2, 2)*accelOut.getZ();
        nums[0][2]=  dcmAtt.getNum(0, 1)*accelOut.getX() + dcmAtt.getNum(1, 1)*accelOut.getY() + dcmAtt.getNum(2, 1)*accelOut.getZ();
        nums[1][0]=  dcmAtt.getNum(0, 2)*accelOut.getX() + dcmAtt.getNum(1, 2)*accelOut.getY() + dcmAtt.getNum(2, 2)*accelOut.getZ();
        nums[1][1]=0;
        nums[1][2]= -dcmAtt.getNum(0, 0)*accelOut.getX() - dcmAtt.getNum(1, 0)*accelOut.getY() - dcmAtt.getNum(2, 0)*accelOut.getZ();
        nums[2][0]= -dcmAtt.getNum(0, 1)*accelOut.getX() - dcmAtt.getNum(1, 1)*accelOut.getY() - dcmAtt.getNum(2, 1)*accelOut.getZ();
        nums[2][1]=  dcmAtt.getNum(0, 0)*accelOut.getX() + dcmAtt.getNum(1, 0)*accelOut.getY() + dcmAtt.getNum(2, 0)*accelOut.getZ();
        nums[2][2]=0;

        Matrix m1=new Matrix(nums);

        return m1.mul(-2);
    }
    private Matrix getErrorB_00(Quaternion attitue){
        return (attitue.con()).getDCM();
    }
    private Matrix getErrorB_01(){
        return new Matrix(3, 0);
    }
    private Matrix getErrorB_02(){
        return new Matrix(3);
    }
    private Matrix getErrorA_10(Quaternion position,double height){
        double[][] nums=new double[3][3];

        Matrix dcmPos=position.getDCM();

        nums[0][0]=-dcmPos.getNum(1, 0);
        nums[0][1]= dcmPos.getNum(0, 0);
        nums[0][2]= 0;
        nums[1][0]=-dcmPos.getNum(1, 1);
        nums[1][1]= dcmPos.getNum(0, 1);
        nums[1][2]= 0;
        nums[2][0]=-dcmPos.getNum(1, 2);
        nums[2][1]= dcmPos.getNum(0, 1);
        nums[2][2]= 0;

        Matrix m1=new Matrix(nums);

        return m1.mul(1/(2*(earthRadius+height)));
    }
    private Matrix getErrorA_11(){
        return new Matrix(3, 0);
    }
    private Matrix getErrorA_12(Quaternion position,Quaternion velocity,double height){
        double[][] nums=new double[3][1];

        Matrix dcmPos=position.getDCM();

        nums[0][0]= dcmPos.getNum(1, 0)*velocity.getX() - dcmPos.getNum(0, 0)*velocity.getY();
        nums[1][0]= dcmPos.getNum(1, 1)*velocity.getX() - dcmPos.getNum(0, 1)*velocity.getY();
        nums[2][0]= dcmPos.getNum(1, 2)*velocity.getX() - dcmPos.getNum(0, 2)*velocity.getY();

        Matrix m1=new Matrix(nums);

        return m1.mul(1/( 2*(earthRadius+height)*(earthRadius+height) ));
    }
    private Matrix getErrorA_13(){
        return new Matrix(3, 0);
    }
    private Matrix getErrorB_10(){
        return new Matrix(3,0);
    }
    private Matrix getErrorB_11(){
        return new Matrix(3,0);
    }
    private Matrix getErrorB_12(){
        return new Matrix(3, 3, 0);
    }
    private Matrix getErrorA_20(){
        double[][] nums=new double[1][3];

        nums[0][0]=0;
        nums[0][1]=0;
        nums[0][2]=1;

        return new Matrix(nums);
    }
    private Matrix getErrorA_21(){
        return new Matrix(1, 3, 0);
    }
    private Matrix getErrorA_22(){
        return new Matrix(1, 1, 0);
    }
    private Matrix getErrorA_23(){
        return new Matrix(1, 3, 0);
    }
    private Matrix getErrorB_20(){
        return new Matrix(1, 3, 0);
    }
    private Matrix getErrorB_21(){
        return new Matrix(1, 3, 0);
    }
    private Matrix getErrorB_22(){
        return new Matrix(1, 3, 0);
    }
    private Matrix getErrorA_30(double height){
        double[][] nums=new double[3][3];

        nums[0][0]=0;
        nums[0][1]=1;
        nums[0][2]=0;
        nums[1][0]=-1;
        nums[1][1]=0;
        nums[1][2]=0;
        nums[2][0]=0;
        nums[2][1]=0;
        nums[2][2]=0;

        Matrix m1=new Matrix(nums);

        return m1.mul(-1/( 2*(earthRadius+height) ));
    }
    private Matrix getErrorA_31(Quaternion position,double height){
        double[][] nums=new double[3][3];

        Matrix dcmPos=position.getDCM();

        nums[0][0]= dcmPos.getNum(0, 1);
        nums[0][1]=-dcmPos.getNum(0, 0);
        nums[0][2]= 0;
        nums[1][0]= dcmPos.getNum(1, 1);
        nums[1][1]=-dcmPos.getNum(1, 0);
        nums[1][2]= 0;
        nums[2][0]= dcmPos.getNum(2, 1);
        nums[2][1]=-dcmPos.getNum(2, 0);
        nums[2][2]= 0;

        Matrix m1=new Matrix(nums);

        return m1.mul(-earthRate.getZ());
    }
    private Matrix getErrorA_32(Quaternion velocity,double height){
        double[][] nums=new double[3][1];

        nums[0][0]= velocity.getY();
        nums[1][0]=-velocity.getX();
        nums[2][0]=0;

        Matrix m1=new Matrix(nums);

        return m1.mul(1/( 2* (earthRadius+height)*(earthRadius+height) ));
    }
    private Matrix getErrorA_33(Quaternion position,Quaternion velocity,double height,Quaternion eToN){
        try {
            double[][] nums = new double[3][3];
            Vector3d w_ei_n = getW_ei_n(eToN);
            Vector3d w_ne_n = getW_ne_n(position, velocity, height);
            Vector3d v1 = w_ei_n.add(w_ne_n);
            nums[0][0] = 0;
            nums[0][1] = -v1.getNum(2);
            nums[0][2] = v1.getNum(1);
            nums[1][0] = v1.getNum(2);
            nums[1][1] = 0;
            nums[1][2] = -v1.getNum(0);
            nums[2][0] = -v1.getNum(1);
            nums[2][1] = v1.getNum(0);
            nums[2][2] = 0;

            Matrix m1 = new Matrix(nums);

            return m1.mul(-1);

        } catch (MatrixInvalidOperationException ex) {
            Logger.getLogger(JavaTestMain.class.getName()).log(Level.SEVERE, null, ex);
            return null;
        }
    }
    private Matrix getErrorB_31(Quaternion attitude){
        double[][] nums = new double[3][3];

        Matrix dcmAtt=attitude.getDCM();

        return dcmAtt.transposition().mul(1.0/2);
    }
    private Matrix getErrorB_30(){
        return new Matrix(3, 3, 0);
    }
    private Matrix getErrorB_32(){
        return new Matrix(3, 3, 0);
    }

    private Quaternion angleToPosition(double lambda,double phi, double alpha){
        //checked.
        double posW= Math.cos((lambda+alpha)/2)*(Math.cos(-phi/2)+Math.sin(-phi/2))/Math.sqrt(2);
        double posX= Math.sin((lambda-alpha)/2)*(Math.cos(-phi/2)-Math.sin(-phi/2))/Math.sqrt(2);
        double posY=-Math.cos((lambda-alpha)/2)*(Math.cos(-phi/2)-Math.sin(-phi/2))/Math.sqrt(2);
        double posZ= Math.sin((lambda+alpha)/2)*(Math.cos(-phi/2)+Math.sin(-phi/2))/Math.sqrt(2);

        return new Quaternion(posW, posX, posY, posZ);
    }
    private double positionToLambda(Quaternion position){
        //checked.
        double lambda=0;

        double posW=position.getW();
        double posX=position.getX();
        double posY=position.getY();
        double posZ=position.getZ();

        lambda=Math.atan2(posZ,posW)-Math.atan2(-posX, -posY);

        if(lambda<-Math.PI){
            lambda+=2*Math.PI;
        }else if(Math.PI<lambda){
            lambda-=2*Math.PI;
        }

        return lambda;
    }
    private double positionToPhi(Quaternion position){
        //checked.
        double phi=0;

        double posW=position.getW();
        double posX=position.getX();
        double posY=position.getY();
        double posZ=position.getZ();

        phi=Math.asin(1-2*(posW*posW+posZ*posZ));

        return phi;
    }
    private double positionToAlpha(Quaternion position){
        //checked.
        double alpha=0;

        double posW=position.getW();
        double posX=position.getX();
        double posY=position.getY();
        double posZ=position.getZ();

        alpha=Math.atan2(posZ, posW)+Math.atan2(-posX, -posY);

        if(alpha<-Math.PI){
            alpha+=2*Math.PI;
        }else if(Math.PI<alpha){
            alpha-=2*Math.PI;
        }

        return alpha;
    }
    private Quaternion updateItoE(Quaternion iToE,double sTimeStep){
        //use constant:earthRate.

        //modified from eqn. B.7.1
        //Quaternion iToN=iToE.mul(eToG).mul(gToN);
        //Quaternion delta=(earthRate.mul(iToN)).mul(0.5);

        Quaternion delta=(earthRate.mul(iToE)).mul(0.5);
        return (iToE.add(delta.mul(sTimeStep))).normalize();
    }
    private Quaternion updateEtoG(Quaternion position){
        //checked.
        double w;
        double x;
        double y;
        double z;

        double lambda=positionToLambda(position);
        double phi=positionToPhi(position);

        w= Math.cos(lambda/2)*(Math.cos(-phi/2)+Math.sin(-phi/2))/Math.sqrt(2);
        x= Math.sin(lambda/2)*(Math.cos(-phi/2)-Math.sin(-phi/2))/Math.sqrt(2);
        y=-Math.cos(lambda/2)*(Math.cos(-phi/2)-Math.sin(-phi/2))/Math.sqrt(2);
        z= Math.sin(lambda/2)*(Math.cos(-phi/2)+Math.sin(-phi/2))/Math.sqrt(2);

        return new Quaternion(w, x, y, z).normalize();
    }
    private Quaternion updateGtoN(Quaternion position){
        //checked.
        double w;
        double z;
        double alpha=positionToAlpha(position);

        w=Math.cos(alpha/2);
        z=Math.sin(alpha/2);

        return new Quaternion(w, 0, 0, z).normalize();
    }
    private Quaternion updateNtoB(Quaternion attitude){
        return attitude.normalize();
    }
    private double getRNormal(Quaternion position){
        double phi = Math.asin(1 - 2 * (position.getW() * position.getW() + position.getZ() * position.getZ()));
        return earthRadius / Math.sqrt(1 - decentering * decentering * Math.sin(phi) * Math.sin(phi));
    }
    private double getRMeridian(Quaternion position){
        double phi = Math.asin(1 - 2 * (position.getW() * position.getW() + position.getZ() * position.getZ()));
        double rMeridian = earthRadius * (1 - decentering * decentering) / Math.pow(1 - decentering * decentering * Math.sin(phi) * Math.sin(phi), 3.0 / 2);
        return rMeridian;
    }
    private Vector3d getW_ne_n(Quaternion position,Quaternion velocity,double height){
        double phi = Math.asin(1 - 2 * (position.getW() * position.getW() + position.getZ() * position.getZ()));
        double alpha = Math.atan2(position.getZ(), position.getW()) + Math.atan2(position.getX(), position.getY());
        double rMeridian=getRMeridian(position);
        double rNormal=getRNormal(position);
        double cosA2 = Math.cos(alpha) * Math.cos(alpha);
        double sinA2 = Math.sin(alpha) * Math.sin(alpha);
        double cosAsinA = Math.sin(alpha) * Math.cos(alpha);
        double w_ne_n_X = velocity.getY() * (cosA2 / (rNormal + height) + sinA2 / (rMeridian + height)) + velocity.getX() * cosAsinA * (1 / (rNormal + height) - 1 / (rMeridian + height));
        double w_ne_n_Y = -velocity.getX() * (cosA2 / (rMeridian + height) + sinA2 / (rNormal + height)) + velocity.getY() * cosAsinA * (1 / (rMeridian + height) - 1 / (rNormal + height));
        Vector3d w_ne_n = new Vector3d(w_ne_n_X, w_ne_n_Y, 0);

        return w_ne_n;
    }
    private Vector3d getW_ei_n(Quaternion eToN){
        return ((eToN.con()).mul(earthRate).mul(eToN)).getVector3d();
    }
    private Quaternion updateVelocity(Quaternion accelOut , Quaternion position, Quaternion velocity, Quaternion attitude , double height,Quaternion eToG,Quaternion gToN,double sTimeStep){
        try {
            //use constant:gravity,earthRate
            //accel.
            Quaternion bToN = attitude.con();
            //colioris
            Quaternion eToN = eToG.mul(gToN);
            Vector3d w_ei_n = getW_ei_n(eToN);
            Vector3d w_ne_n=getW_ne_n(position, velocity, height);
            Vector3d vectColioris = (w_ei_n.mul(2.0).add(w_ne_n)).cross(velocity.getVector3d());
            Quaternion colioris=new Quaternion(0, vectColioris);
            //centrifugal
            double rNormal=getRNormal(position);
            double scale=2*earthRate.getZ()*earthRate.getZ()*(rNormal+height);
            double centX=position.getW()*position.getY()+position.getX()*position.getZ();
            double centY=position.getZ()*position.getY()-position.getX()*position.getW();
            Quaternion centrifugal= new Quaternion(0, centX, centY, 0).mul(scale);

            //delta
            Quaternion delta=(bToN.con().mul(accelOut).mul(bToN))
              .add(gravity)
              .sub(colioris)
              .sub(eToN.con().mul(centrifugal).mul(eToN));

            return velocity.add(delta.mul(sTimeStep));

        } catch (MatrixInvalidOperationException ex) {
            Logger.getLogger(JavaTestMain.class.getName()).log(Level.SEVERE, null, ex);
        }
        return null;
    }
    private Quaternion updatePosition(Quaternion position,Quaternion velocity, double height,Quaternion eToG,Quaternion gToN,double sTimeStep){
        Quaternion eToN   = eToG.mul(gToN);
        Quaternion w_ne_n = new Quaternion(0, getW_ne_n(position, velocity, height));
        Quaternion delta=(eToN.mul(w_ne_n)).mul(0.5);
        return (position.add(delta.mul(sTimeStep))).normalize();
    }
    private double updateHeight(double height,Quaternion velocity,double sTimeStep){
        double delta=-(velocity.getZ());
        return height+delta*sTimeStep;
    }
    private Quaternion updateAttitude( Quaternion gyroOut,Quaternion position,Quaternion velocity, double height,Quaternion attitude,Quaternion eToG,Quaternion gToN,double sTimeStep){
        Quaternion eToN=eToG.mul(gToN);
        Quaternion w_ei_n=(eToN.con()).mul(earthRate).mul(eToN);
        Quaternion w_ne_n=new Quaternion(0, getW_ne_n(position, velocity, height));

        Quaternion delta=(attitude.mul(gyroOut)
                .sub((w_ei_n.add(w_ne_n)).mul(attitude))).mul(0.5);

        return (attitude.add(delta.mul(sTimeStep))).normalize();
    }

    static double[] lowPassFilter(double[] data,double cuttoff){
        double[] filtered = new double[data.length];
        double w;
        w=2*Math.tan(Math.PI*cuttoff/2);
        filtered[0]=(w*data[0])/(w+2);
        for (int i = 1; i < data.length; i++) {
            filtered[i]=(w*data[i]+w*data[i-1]+(2-w)*filtered[i-1])/(w+2);
        }
        return filtered;
    }
    static String[] exec(String command) {
        String [] strs;
        ArrayList<String> tmp = new ArrayList<String>();
        try {
            Runtime runtime = Runtime.getRuntime();
            Process process;
            process = runtime.exec(command);

            InputStream is = process.getInputStream();
            BufferedReader br = new BufferedReader(new InputStreamReader(is));
            InputStream err = process.getErrorStream();
            BufferedReader errbr = new BufferedReader(new InputStreamReader(err));
            String line;
            while ((line = br.readLine()) != null) {
                tmp.add(line);
                //System.out.println(line);
            }
            while ((line = errbr.readLine()) != null) {
                System.out.println(line);
            }

        } catch (IOException ex) {
            System.out.println("excep");
        }
        strs=new String[tmp.size()];
        for (int i = 0; i < tmp.size(); i++) {
            strs[i]=tmp.get(i);
        }
        return strs;
    }
}