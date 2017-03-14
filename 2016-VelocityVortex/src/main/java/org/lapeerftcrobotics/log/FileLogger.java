package org.lapeerftcrobotics.log;

import android.graphics.Bitmap;
import android.os.Environment;
import android.util.Log;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.File;
import java.io.FileOutputStream;
import java.io.FileWriter;

/**
 * Created by T6810SM on 9/9/2015.
 */
public class FileLogger {

    private final static int BUFFER_SIZE = 10;
    private String[] buffer = new String[BUFFER_SIZE];
    private int index = 0;
    private FileWriter writer;
    private String filename;
    private ElapsedTime elapsedTime;
    private String filenamePrefix = "";
    private boolean isOpen = false;

    public FileLogger(ElapsedTime elapsedTime) {
        this.elapsedTime = elapsedTime;
    }

    public String getFilename() {
        return this.filename;
    }

    public String getFilenamePrefix() { return this.filenamePrefix; }

    public boolean isExternalStorageWritable() {
        String state = Environment.getExternalStorageState();
        if (Environment.MEDIA_MOUNTED.equals(state)) {
            return true;
        }
        return false;
    }

    public File getStorageDir(String fileName) {
        // Get the directory for the user's public docs directory.
        File file = new File(Environment.getExternalStoragePublicDirectory(
                Environment.DIRECTORY_DOCUMENTS), fileName);
        return file;
    }

    public void open() {
        String outFile = "";
        try {
            long tm = System.currentTimeMillis();
            this.filenamePrefix = ""+tm;
            String fileName = "rlog"+tm+".txt";
            if (isExternalStorageWritable()) {
                File out = getStorageDir(fileName);
                writer = new FileWriter(out.getAbsolutePath(),true);
                if (out != null) {
                    this.filename = out.toString();
                    System.out.println("Opened file: "+out.toString());
                    isOpen = true;
                }
            }
            else {
                System.out.println("External Storage is not writable...");
            }
        } catch( Exception ex) {
            Log.e("Err","Caught Exception opening file: "+outFile+", ex: "+ex);
        }
    }

    public void saveBitmap(Bitmap bm, String filename)
    {
        String fileName = this.filenamePrefix + filename;
        try {
            if (isExternalStorageWritable()) {
                File out = getStorageDir(fileName);
                FileOutputStream stream = new FileOutputStream(out);
                bm.compress(Bitmap.CompressFormat.JPEG, 100, stream);
                stream.flush();
                stream.close();
            }
            else {
                System.out.println("External Storage is not writable...");
            }
        } catch( Exception ex) {
            Log.e("Err","Caught Exception opening file: "+fileName+", ex: "+ex);
        }
    }

    public void close() {
        try {
            writeBuffer();
            writer.flush();
            writer.close();
        } catch( Exception ex) {
            System.out.println("Caught Exception closing file: "+ex);
        }
        isOpen = false;
    }

    public synchronized void writeEvent(String event, String desc) {
        if (isOpen)
            this.write(this.elapsedTime.toString()+","+System.currentTimeMillis()+","+Thread.currentThread().getId()+","+event+","+desc);
    }

    public synchronized void write(String line) {
        if (isOpen) {
            if (index == BUFFER_SIZE) {
                writeBuffer();
            }
            buffer[index] = line;
            index++;
        }
    }

    private void writeBuffer() {
        for (int i=0; i<buffer.length; i++) {
            try {
                if (writer != null) {
                    writer.flush();
                    if (buffer[i] != null)
                        writer.write(buffer[i]);
                    writer.write('\r');
                    writer.write('\n');
                }
            } catch( Exception ex) {
                System.out.println("Caught Exception writing buffer["+i+"] value: "+ex);
            }
        }
        index = 0;
    }
}
