#ifndef VOXELGRAPH_H
#define VOXELGRAPH_H

#ifdef __cplusplus
extern "C"{
#endif

#include "Chunk.h"
#include <stdlib.h>
#include <assert.h>
#include <stdbool.h>
typedef struct{
    uint32_t chunk_amount;
    uint32_t chunk_hash_table_size;
    uint32_t current_chunk_index;
    Chunk_t* chunks;
    int32_t* chunk_hash_table;
}VoxelGraph_t;

VoxelGraph_t* voxel_graph_init();
void voxel_graph_free(VoxelGraph_t** graph);
bool voxel_graph_insert(VoxelGraph_t* graph, int64_t x, int64_t y, int64_t z);
int64_t voxel_graph_chunk_hash_table_request(VoxelGraph_t* graph, int64_t x, int64_t y, int64_t z);
int64_t voxel_graph_chunk_hash_table_lookup(VoxelGraph_t* graph, int64_t x, int64_t y, int64_t z);
int64_t voxel_graph_create_chunk(VoxelGraph_t* graph, int64_t x, int64_t y, int64_t z);
#ifdef __cplusplus
}
#endif

#endif

