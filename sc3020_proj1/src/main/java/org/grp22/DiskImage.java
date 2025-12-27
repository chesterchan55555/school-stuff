package org.grp22;
import java.io.*;
import java.util.List;

public class DiskImage {
    private static final int BLOCK_SIZE = Config.BLOCK_SIZE;
    private RandomAccessFile disk;
    private int numOfBlocks;
    private int totalRecords;
    private int dataBlocks;
    private static final int RECORD_SIZE = Config.RECORD_SIZE;
    private static final int RECORDS_PER_BLOCK = Config.RECORDS;

    public DiskImage(String fileName, int sizeMB) throws IOException{
        File f = new File("project_data/"+fileName);
        if(!f.exists()){
            try(FileOutputStream fos = new FileOutputStream(f)){
                byte[] zeroes = new byte[1024 * 1024]; // 1MB Buffer
                for (int i = 0; i<sizeMB; i++){
                    fos.write(zeroes);
                }
            }
        }
        this.disk = new RandomAccessFile(f, "rw");
        this.numOfBlocks = (int) (f.length() / BLOCK_SIZE);
        this.totalRecords = 0;
        this.dataBlocks = 0;
    }

    public byte[] readBlock(int index) throws IOException{
        byte[] buffer = new byte[BLOCK_SIZE];
        disk.seek((long) index * BLOCK_SIZE);
        disk.readFully(buffer);
        return buffer;
    }

    public void writeBlock(int index, byte[] data) throws IOException{
        if (data.length > BLOCK_SIZE) throw new IllegalArgumentException("Data Larger than Block");
        disk.seek((long) index * BLOCK_SIZE); //Seek position of indexed block
        disk.write(data);
    }

    public void insertRecords(List<DataCleaner.Record> records) throws IOException {
        System.out.println("Storing " + records.size() + " records on disk...");

        this.totalRecords = records.size();
        this.dataBlocks = (int) Math.ceil((double) totalRecords / RECORDS_PER_BLOCK);

        int currentBlock = 0;
        int recordsInCurrentBlock = 0;
        byte[] blockBuffer = new byte[BLOCK_SIZE];
        int bufferPosition = 0;

        for (DataCleaner.Record record : records) {
            byte[] recordBytes = record.toBytes();

            // If current block is full, write it to disk and start new block
            if (recordsInCurrentBlock >= RECORDS_PER_BLOCK) {
                writeBlock(currentBlock, blockBuffer);
                currentBlock++;
                blockBuffer = new byte[BLOCK_SIZE];
                bufferPosition = 0;
                recordsInCurrentBlock = 0;
            }

            // Add record to current block buffer
            System.arraycopy(recordBytes, 0, blockBuffer, bufferPosition, RECORD_SIZE);
            bufferPosition += RECORD_SIZE;
            recordsInCurrentBlock++;
        }

        // Write the last partially filled block
        if (recordsInCurrentBlock > 0) {
            writeBlock(currentBlock, blockBuffer);
        }

        System.out.println("Data storage complete. Used " + dataBlocks + " blocks.");
    }

    public void printDiskInformation() {
        System.out.println("\n=== TASK 1 RESULTS ===");
        System.out.println("Number of records: " + totalRecords);
        System.out.println("Size of a record: " + RECORD_SIZE + " bytes");
        System.out.println("Number of records stored in a block: " + RECORDS_PER_BLOCK);
        System.out.println("Number of blocks storing the data: " + dataBlocks);

        // Additional useful information
        System.out.println("\n=== ADDITIONAL DETAILS ===");
        System.out.println("Block size: " + BLOCK_SIZE + " bytes");
        System.out.println("Total disk space used for data: " + (dataBlocks * BLOCK_SIZE) + " bytes");
        System.out.println("Data utilization: " + String.format("%.2f%%", (double)(totalRecords * RECORD_SIZE) / (dataBlocks * BLOCK_SIZE) * 100));
    }

    public int getBlockCount(){
        return numOfBlocks;
    }

    public int getTotalRecords() {
        return totalRecords;
    }

    public int getBlocksUsedForData() {
        return dataBlocks;
    }

    public void close() throws IOException{
        disk.close();
    }
}
