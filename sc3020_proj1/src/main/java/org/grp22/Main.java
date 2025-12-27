package org.grp22;

import java.io.*;
import java.util.List;
import org.grp22.DiskImage;
import org.grp22.DataCleaner;

public class Main {
    private static DiskImage disk;

    public static void main(String[] args) throws IOException {
        System.out.println("Setting up disk and loading NBA game data...");

        // Initialize disk
        disk = new DiskImage("grp22Disk.img", Config.DISK_SIZE / Config.MB);

        // Load and clean the data
        List<DataCleaner.Record> records = DataCleaner.clean("project_data/games.txt");

        // Store data on disk
        disk.insertRecords(records);

        // Run experiment 1
        runExp1();

        disk.close();
    }

    public static void runExp1() {
        System.out.println("Running Experiment 1...");
        disk.printDiskInformation();
        System.out.println("Experiment 1 finished");
    }
}
