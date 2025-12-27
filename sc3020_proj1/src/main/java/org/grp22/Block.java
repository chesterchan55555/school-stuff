package org.grp22;

public interface Block {
    byte TYPE_LEAF = 1;
    byte TYPE_INTERNAL = 2;
    byte TYPE_DATA = 3;

    byte getBlockType();  // each block returns its type
    byte[] toBytes();     // serialize block to byte array
    
}

