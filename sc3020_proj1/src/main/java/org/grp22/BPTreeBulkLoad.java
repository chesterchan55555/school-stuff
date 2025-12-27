package org.grp22;

import static org.grp22.Config.*;
import java.io.*;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;
import java.util.*;
import org.grp22.TreeNodes.*;

public class BPTreeBulkLoad {

    // Constants
    public static final int BLOCK_SIZE = 4096;

    public static final int RECORD_SIZE = 22;
    public static final int LEAF_ENTRY_SIZE = 6;       // key(short 2)+pointer(int 4)
    public static final int INTERNAL_ENTRY_SIZE = 6;   // key(short)+pointer(int)
    public static final int MAX_INTERNAL_ENTRIES = (BLOCK_SIZE - 1- 4) / INTERNAL_ENTRY_SIZE; //4 bytes for  0th pointer size, number of keys.
    public static final int MAX_LEAF_ENTRIES = (BLOCK_SIZE - 1 - 4) / LEAF_ENTRY_SIZE; //4 bytes for pointer to next leaf node
    public static final int MAX_DATA_RECORDS_PER_BLOCK = (BLOCK_SIZE-1) / RECORD_SIZE; //187 records per data block
    public static final byte TYPE_DATA = 1;
    public static final byte TYPE_LEAF = 2;
    public static final byte TYPE_INTERNAL = 3;

/*
    // ----------------- DataBlock -----------------
    static class DataBlock {
        List<DataCleaner.Record> records = new ArrayList<>();
        public byte[] toBytes() {
            ByteBuffer buffer = ByteBuffer.allocate(BLOCK_SIZE);
            for (DataCleaner.Record r : records) buffer.put(r.toBytes());
            return buffer.array();
        }
    }

    // ----------------- LeafNode -----------------
    static class LeafNode {
        List<Short> keys = new ArrayList<>();
        List<Integer> dataPointers = new ArrayList<>();
        int nextLeaf = -1;
// why -1??
        public byte[] toBytes() {
            ByteBuffer buffer = ByteBuffer.allocate(BLOCK_SIZE);
            buffer.put(TYPE_LEAF);        // <--- block type first
            buffer.putInt(nextLeaf);
            for (int i = 0; i < keys.size(); i++) {
                buffer.putShort(keys.get(i));
                buffer.putInt(dataPointers.get(i));
            }
            return buffer.array();
        }
    }

    // ----------------- InternalNode -----------------
    static class InternalNode {
        List<Short> keys = new ArrayList<>();
        List<Integer> childPointers = new ArrayList<>();

        public byte[] toBytes() {
            ByteBuffer buffer = ByteBuffer.allocate(BLOCK_SIZE);
            buffer.put(TYPE_INTERNAL);        // <--- block type first
            for (int i = 0; i < keys.size(); i++) {
                buffer.putShort(keys.get(i));
                buffer.putInt(childPointers.get(i));
            }
            return buffer.array(); //doesnt include last pointer, ptr needs to start from 0, but key starts from 1.
        }
    }

    // ----------------- Write block -----------------
    static int writeBlock(FileChannel fc, byte[] blockBytes) throws IOException {
        long position = fc.size(); //appends to end of file, so gets the position of the end
        fc.position(position);
        fc.write(ByteBuffer.wrap(blockBytes));
        return (int)(position / BLOCK_SIZE);
    }
    */
    // --Recursive builder--
    static int buildTree(FileChannel fc, List<Integer> childBlockIds, List<Short> separatorKeys) throws IOException {
        // If only one child block, it’s the root

        List<Integer> parentBlockIds = new ArrayList<>();
        List<Short> parentSeparatorKeys = new ArrayList<>();

        int i = 0;
        while (i < childBlockIds.size()) {
            InternalNode node = new InternalNode();
            int count = 0;

            // Fill node up to capacity
            while (count < MAX_INTERNAL_ENTRIES && i < childBlockIds.size()) {
                node.childPointers.add(childBlockIds.get(i));

                // Add separator key for all but the first child
                if (count > 0) {
                    node.keys.add(separatorKeys.get(i));
                    System.out.println(separatorKeys.get(i)); //need to fix this. wrong keys in internal nodes.
                }

                i++;
                count++;
            }
            // Write internal node to disk
            int blockId = BlockIO.writeBlock(fc, node);
            parentBlockIds.add(blockId);

            // if internal node is root, means only have 1 parent block
            if (parentBlockIds.size() == 1) {
                System.out.println("root key IDs: " );
                for (int j = 0; j < separatorKeys.size(); j++) {
                    System.out.println(j); // prints number of j
                }
                return blockId;
            }
            // Use first key of this node’s children as separator for higher level
            if (!node.keys.isEmpty()) {
                parentSeparatorKeys.add(separatorKeys.get(i-count)); //wrong coz need to find smallest key in the subtree.
            }
        }

        // Recurse to build next level
        return buildTree(fc, parentBlockIds, parentSeparatorKeys);
    }

    public static void main(String[] args) throws Exception {
        // 1. Cleaned records
        List<DataCleaner.Record> records = DataCleaner.clean("project_data/games.txt");

        // 2. Sort by FT_PCT_home
        records.sort(Comparator.comparingInt(r -> r.ftPct));

        try (RandomAccessFile raf = new RandomAccessFile("disk.img", "rw");
             FileChannel fc = raf.getChannel()) {

            // 3. Data blocks
            List<Integer> dataBlockIds = new ArrayList<>();
            int i = 0;
            while (i < records.size()) {
                DataBlock db = new DataBlock();
                for (int j = 0; j < MAX_DATA_RECORDS_PER_BLOCK && i < records.size(); j++, i++) // records capacity of data blks
                    db.records.add(records.get(i));
                int blockId = BlockIO.writeBlock(fc, db);
                dataBlockIds.add(blockId);
            }
            System.out.println("Data blocks written: " + dataBlockIds.size());

            // 4. Leaf nodes with direct record addresses regardless of which data blk
            List<Short> leafSeparatorKeys = new ArrayList<>();
            List<Integer> leafBlockIds = new ArrayList<>();
            int recordIndex = 0;
            LeafNode leaf = new LeafNode();
            Integer prevLeafBlockId = null;
            // fills up leaves based on data blocks.
            for (int dbIndex = 0; dbIndex < dataBlockIds.size(); dbIndex++) {
                int DataBlock = dataBlockIds.get(dbIndex); // in case file already has other data.
                for (int j = 0; j < MAX_DATA_RECORDS_PER_BLOCK && recordIndex < records.size(); j++, recordIndex++) {
                    DataCleaner.Record r = records.get(recordIndex);

                    // Add record key and addr to leaf
                    leaf.keys.add(r.ftPct);
                    int recordAddr = (DataBlock * BLOCK_SIZE) + (j * RECORD_SIZE); //datablk addr + offset
                    leaf.recordAddresses.add(recordAddr);

                    // Check if leaf is full
                    if (leaf.keys.size() == MAX_LEAF_ENTRIES) {
                        int leafBlockId = BlockIO.writeBlock(fc, leaf);

                        // Link previous leaf
                        if (prevLeafBlockId != null) {
                            updateLeafNext(fc, prevLeafBlockId, leafBlockId);
                        }
                        prevLeafBlockId = leafBlockId;

                        leafBlockIds.add(leafBlockId);
                        leafSeparatorKeys.add(leaf.keys.get(0)); // first key as separator for internal node
                        leaf = new LeafNode(); // start new leaf
                    }
                }
            }
            // Write last leaf if it has any records
            if (!leaf.keys.isEmpty()) {
                int leafBlockId = BlockIO.writeBlock(fc, leaf);
                if (prevLeafBlockId != null) {
                    updateLeafNext(fc, prevLeafBlockId, leafBlockId);
                }
                leafBlockIds.add(leafBlockId);
                leafSeparatorKeys.add(leaf.keys.get(0));
            }
            System.out.println("Leaf nodes written: " + leafBlockIds.size());

            // 5. Build internal nodes and root

            int rootBlockId = buildTree(fc, leafBlockIds, leafSeparatorKeys);

            System.out.println("B+ tree bulk load complete. Root block ID: " + rootBlockId);
        }
    }
    // Update nextLeaf pointer in leaf node at leafBlockId, Helper function
    static void updateLeafNext(FileChannel fc, int leafBlockId, int nextLeafBlockId) throws IOException {
    long pos = (long) leafBlockId * BLOCK_SIZE + 1; // +1 to skip block_type
    ByteBuffer buf = ByteBuffer.allocate(4);
    buf.putInt(nextLeafBlockId);
    buf.flip();
    fc.position(pos);
    fc.write(buf);
    }
}
