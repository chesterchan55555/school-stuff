package org.grp22;

public class Config {
    private Config() {
            // restrict instantiation
    }
    // Disk and Memory Configuration
    public static final int MB = 1 << 20; // 1,048,576 bytes
    public static final int DISK_SIZE = 100 * MB;

    // Block and Record Configuration
    public static final int BLOCK_SIZE = 4096;
    public static final int RECORD_SIZE = 22;
    public static final int RECORDS = BLOCK_SIZE / RECORD_SIZE;

    // B+ Tree Configuration
    public static final int LEAF_ENTRY_SIZE = 6;       // key(short)+pointer(int)
    public static final int INTERNAL_ENTRY_SIZE = 6;   // key(short)+pointer(int)
    public static final int MAX_INTERNAL_ENTRIES = (BLOCK_SIZE - 4 - 1) / INTERNAL_ENTRY_SIZE; //681, 6 bytes for  0th pointer size, number of keys.
    public static final int MAX_LEAF_ENTRIES = (BLOCK_SIZE - 4 - 1) / LEAF_ENTRY_SIZE; //6 bytes for pointer to next leaf node
    public static final int MAX_DATA_RECORDS_PER_BLOCK = (BLOCK_SIZE - 1) / RECORD_SIZE; // 186 records per data block , 1 for data block type

    // Block Type Constants
    public static final byte TYPE_DATA = 1;
    public static final byte TYPE_LEAF = 2;
    public static final byte TYPE_INTERNAL = 3;
}
