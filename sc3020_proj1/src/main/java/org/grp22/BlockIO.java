package org.grp22;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;
import org.grp22.TreeNodes.*;
import static org.grp22.Config.*;


//read, write, delete
public class BlockIO {


    // Constants
    public static final int BLOCK_SIZE = 4096;


    // ---------- Read ----------
    public static Object readNode(FileChannel fc, int blockId) throws IOException {
        ByteBuffer buffer = ByteBuffer.allocate(BLOCK_SIZE);
        fc.position((long) blockId * BLOCK_SIZE);
        fc.read(buffer);
        buffer.flip();

        // Read type byte
        byte type = buffer.get();

        switch (type) {
            case TYPE_DATA:
                return DataBlock.fromBytes(buffer);

            case TYPE_LEAF:
                return LeafNode.fromBytes(buffer);

            case TYPE_INTERNAL:
                return InternalNode.fromBytes(buffer);

            default:
                throw new IOException("Unknown block type " + type + " at block " + blockId);
        }
    }

    // ---------- Write ----------
    public static int writeBlock(FileChannel fc, Block block) throws IOException {
        byte[] data = block.toBytes(); // done here
        ByteBuffer buffer = ByteBuffer.wrap(data);

        long fileSize = fc.size();
        int blockId = (int) (fileSize / BLOCK_SIZE);

        fc.position((long) blockId * BLOCK_SIZE);
        fc.write(buffer);

        return blockId;
    }

}
