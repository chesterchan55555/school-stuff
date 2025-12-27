package org.grp22;

public class BTreeConstructionFlow {
    //size of block 
    // per block: 
    //number of data entries, size of each data entry
    // size of each pointer and each key. affects number of keys/pointers per block
    // 26652 entries so 
    //given that the task only includes deletion and no insertion, can fill up each block.

    //key is FT_PCT_home 
/*constants for data storage:
 * no. of data blocks = ceil(26652 * (28+header) / 4096) = 183 (assuming header is negligible)
 * 
 * 
 * 
 * constants fr b+ tree construction:
 * block size = 4096 bytes (because matches the OS page size on most systems))
 * pointer size = 6 bytes
 * key size = 2 bytes
 * size of an entry = 28 bytes (10+4+2+2+2+2+2+2+1+1, based on schema)
 * max no of key-pointer pairs per leaf node = floor((4096-6)/8) = 511 (6 bytes for pointer to next leaf node)
 * internal node = floor(4096-6/8) = 511 (since contains n+1 pointers and n key entries in a B+ Tree internal node)
 * no. of leaf nodes = ceil(26652/511) = 53
 * 
 * 
 */ 
/*
 * Pseudocode for construction (bulk loading) + storing data entries in disk.img:
 * 1. Sort the data entries based on the key (FT_PCT_home) via quick sort.
 * 2. Group the data entries into blocks of 146 entries each (since 4096/28 = 146). 
 * 3. For each block, create a leaf node in the B+ tree and pack the block's entries in it.
 * 3a.Pack the key entries along with their pointers to their data entry in leaf nodes, (4096-6)/8 = 511 key entry + pointer pair per leaf node.
 * 3b. Get a pointer for each key entry to its corresponding data entry address.
 * 4. For every leaf node, store a pointer to the next leaf node to facilitate range queries.
 * 5. Once all leaf nodes are created, move up to create internal nodes.
 * 6. For internal nodes, each can hold up to 511 key-pointer pairs (since contains n+1 pointers and n key entries in a B+ Tree internal node).
 * 7. For each internal node, starting from the first leaf node, create a pointer in the internal node to the first leaf node and use the first key of each subsequent leaf node as the key in the internal node and create a pointer to that leaf node in the internal node.
 * 8. Repeat this process until all leaf nodes are covered, creating new internal nodes as needed, ensure each key in a key-pointer pair is the smallest key to be found in the subtree pointed to by its corresponding pointer.
 * 9. Continue this process of creating higher-level internal nodes until reaching the root node, which will point to a single node at the top of the tree.
 * 10. Write each node (leaf and internal) to the disk image file in a sequential manner, ensuring that each node is stored in a block of 4096 bytes.
 * 11. Any unused space in a block can be padded with zeros to maintain the block size.
 * 12. Maintain address of root node for any queries, which allows traversal from the root to the leaves and access to the data entries.
 * 
 * 
 * TASK 1 stats:
 * record: 28 bytes
 * number of records: 26652
 * number of blocks for storing data: 183 + 53 + 1 = 237 blocks
 * 
 * TASK 2 stats:
 * n = 511, given 4096 bytes block size, 6 bytes pointer size, 2 bytes key size
 * nodes in B tree = 53 + 1 = 54 nodes
 * number of levels = 2 (1 root + 1 leaf level)
 * content of root node(key values)=  138 (first key of second leaf node)
 * 
 * 
 */

}
