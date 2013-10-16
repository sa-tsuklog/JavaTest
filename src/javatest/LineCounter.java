/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package javatest;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;

/**
 *
 * @author sa
 */
public class LineCounter {

    String filePostfix[] = {".cpp",".hpp","c","h"};
    int fileCount;
    int lineCount;
    int charCount;
    
    /**
     * @param args the command line arguments
     */
    public static void main(String[] args) {
        LineCounter lc = new LineCounter();
    }

    public int countLine(File file){
        int count=0;
        try {
            BufferedReader br = new BufferedReader(new FileReader(file));
            String line;
            while((line = br.readLine()) != null){
                count++;
                charCount += line.length();
            }
            br.close();
        } catch (IOException iOException) {
        }
        return count;
    }
    
    public int count(File file){
        if(file.isDirectory()){
            System.out.println("directory:"+file.getName());
            String[] fileList = file.list();
            for (int i = 0; i < fileList.length; i++) {
                count(new File(file.getAbsolutePath()+"\\"+fileList[i]));
            }
        }else if(file.isFile()){
            for (int i = 0; i < filePostfix.length; i++) {
                if(file.getName().matches(".+"+filePostfix[i]+"$")){
                    fileCount++;
                    int tmp = countLine(file);
                    System.out.println(file.getName()+"\t"+tmp+"\tlines");
                    lineCount += tmp;
                }else{
                }
            }
        }
        return 0;
    }
    public LineCounter() {
        lineCount = 0;
        fileCount = 0;
        charCount = 0;
        count(new File("E:\\MyDocument\\Dropbox\\EmbededProjects\\62N_PBGlider\\src\\Peripherals"));
        
        System.out.println("file num:"+fileCount);
        System.out.println("total line num:"+lineCount);
        System.out.println("char count:"+ charCount);
    }
    
}
