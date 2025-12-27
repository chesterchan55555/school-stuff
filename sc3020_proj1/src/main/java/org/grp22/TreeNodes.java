package org.grp22;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;
import static org.grp22.Config.*;


// blockIO gets the 1 byte block type from the start of the block, main file ensures no overflow.
public class TreeNodes {
    //constants
     public static final int BLOCK_SIZE = 4096;
     public static final int RECORD_SIZE = 22;
    public static final int LEAF_ENTRY_SIZE = 6;       // key(short)+pointer(int)
    public static final int INTERNAL_ENTRY_SIZE = 6;   // key(short)+pointer(int)

    static class DataBlock implements Block {
        List<DataCleaner.Record> records = new ArrayList<>();

        @Override
        public byte getBlockType() { return TYPE_DATA; }

        @Override
        public byte[] toBytes() {
            ByteBuffer buffer = ByteBuffer.allocate(BLOCK_SIZE);
            buffer.put(getBlockType()); // 1 byte block type

            for (DataCleaner.Record r : records) {
                buffer.put(r.toBytes());
            }
            return buffer.array();
        }

        public static DataBlock fromBytes(ByteBuffer buffer) {
            DataBlock db = new DataBlock();
            while (buffer.remaining() >= RECORD_SIZE) { // each record = 22 bytes
                db.records.add(DataCleaner.Record.fromBytes(buffer));
            }
            return db;
        }
    }
    static class LeafNode implements Block {
        List<Short> keys = new ArrayList<>();
        List<Integer> recordAddresses = new ArrayList<>(); // 4-byte record addresses
        int nextLeaf = -1; // -1 means this is the last leaf (no next leaf)

        @Override
        public byte getBlockType() { return TYPE_LEAF; }

        @Override
        public byte[] toBytes() {
            ByteBuffer buffer = ByteBuffer.allocate(BLOCK_SIZE);
            buffer.put(getBlockType());   // 1 byte block type
            buffer.putInt(nextLeaf);      // 4 bytes next leaf pointer

            for (int i = 0; i < keys.size(); i++) {
                buffer.putShort(keys.get(i));           // 2 bytes
                buffer.putInt(recordAddresses.get(i)); // 4 bytes
            }
            return buffer.array();
        }

        public static LeafNode fromBytes(ByteBuffer buffer) { // the 1byte is alr removed
            LeafNode leaf = new LeafNode();
            leaf.nextLeaf = buffer.getInt();// read next leaf pointer first

            while (buffer.remaining() >= LEAF_ENTRY_SIZE ) { // 2 + 4 bytes per entry
                leaf.keys.add(buffer.getShort());
                leaf.recordAddresses.add(buffer.getInt());
            }
            return leaf;
        }
    }
    static class InternalNode implements Block {
        List<Short> keys = new ArrayList<>();
        List<Integer> childPointers = new ArrayList<>(); // block IDs

        @Override
        public byte getBlockType() { return TYPE_INTERNAL; }

        @Override
        public byte[] toBytes() {
            ByteBuffer buffer = ByteBuffer.allocate(BLOCK_SIZE);
            buffer.put(getBlockType()); // 1 byte block type

            for (int i = 0; i < keys.size(); i++) {
                buffer.putShort(keys.get(i));         // 2 bytes
                buffer.putInt(childPointers.get(i));  // 4 bytes
            }

            // IMPORTANT: if you want a trailing child pointer (last pointer) uncomment:
            // buffer.putInt(childPointers.get(childPointers.size() - 1));

            return buffer.array();
        }

        public static InternalNode fromBytes(ByteBuffer buffer) {
            InternalNode node = new InternalNode();
             // First child pointer
            if (buffer.remaining() >= 4) {
                node.childPointers.add(buffer.getInt());
            }

            while (buffer.remaining() >= INTERNAL_ENTRY_SIZE) { // short + int
                node.keys.add(buffer.getShort());
                node.childPointers.add(buffer.getInt());
            }
            return node;
        }
    }



}
