#include "VoxelGraph.h"
#include <assert.h>
#include "HashUtils.h"
#include <stdio.h>
//helper functions
void voxel_graph_init_arrays(VoxelGraph_t* graph){
    assert(graph != NULL);
    for(uint32_t i = 0; (i < 1<<15) && (i < graph->chunk_amount); i++){
        graph->chunks[i] = chunk_init();
    }
    uint32_t ref_val = (uint32_t) 1 << 31;
    for(uint32_t i = 0; (i < ref_val) && (i < graph->chunk_hash_table_size);i++){
        graph->chunk_hash_table[i] = -1;
    }
    assert(graph->chunks[0].x_offset == 0);
    assert(graph->chunks[0].y_offset == 0);
    assert(graph->chunks[0].z_offset == 0);
    assert(graph->chunks[0].current_node_index == 0);
    
    assert(graph->chunks[graph->chunk_amount - 1].x_offset == 0);
    assert(graph->chunks[graph->chunk_amount - 1].y_offset == 0);
    assert(graph->chunks[graph->chunk_amount - 1].z_offset == 0);
    assert(graph->chunks[graph->chunk_amount - 1].current_node_index == 0);

    assert(graph->chunk_hash_table[0] == -1);
    assert(graph->chunk_hash_table[graph->chunk_hash_table_size - 1] == -1);
}

//end helpers
VoxelGraph_t* voxel_graph_init(uint32_t chunk_count)
{
    assert(chunk_count > 0);
    assert((chunk_count & (chunk_count - 1)) == 0);
    uint32_t chunk_amount = chunk_count;
    uint32_t chunk_hash_table_size = CHUNK_HASH_TABLE_SIZE;
    VoxelGraph_t* output = malloc(sizeof(VoxelGraph_t));
    output->chunks = NULL;
    output->chunk_hash_table = NULL;
    output->current_chunk_index = 0;
    for(uint32_t i = 0; i < 31 && output->chunks == NULL; i++){
        unsigned long int init_size = sizeof(Chunk_t) * chunk_amount;
        output->chunks = malloc(init_size);
        if(output->chunks == NULL){
            chunk_amount = chunk_amount >> 1;
        }
        else{
            output->chunk_amount = chunk_amount;
        }
    }
    assert(output->chunks != NULL);
    for(uint32_t i = 0; i < 31 && output->chunk_hash_table == NULL; i++){
        output->chunk_hash_table = malloc(sizeof(uint32_t) * chunk_hash_table_size);
        if(output->chunk_hash_table == NULL){
            chunk_hash_table_size = chunk_hash_table_size >> 1;
        }
        else{
            output->chunk_hash_table_size = chunk_hash_table_size;
        }
    }
    printf("chunk amount is %d\n", output->chunk_amount);
    printf("chunk hashtable size is %d\n", output->chunk_hash_table_size);
    assert(output->chunk_hash_table != NULL);
    assert(output->chunk_hash_table_size > output->chunk_amount);
    voxel_graph_init_arrays(output);
    return output;
}

bool voxel_graph_insert(VoxelGraph_t* graph, int64_t x, int64_t y, int64_t z){
    assert(graph != NULL);
    //printf("performing hash table lookup\n");
    int64_t arr_entry = voxel_graph_chunk_hash_table_request(graph, x, y, z);
    assert(arr_entry < graph->chunk_amount);
    if(arr_entry < 0){
        return false;
    }
    assert(graph->chunks[arr_entry].x_offset == build_anchor_coord(x));
    assert(graph->chunks[arr_entry].y_offset == build_anchor_coord(y));
    assert(graph->chunks[arr_entry].z_offset == build_anchor_coord(z));
    //printf("performing chunk_insertion\n");
    bool ret_val = chunk_insert(&graph->chunks[arr_entry],x, y, z);
    return ret_val;
}
int64_t voxel_graph_chunk_hash_table_lookup(VoxelGraph_t* graph, int64_t x, int64_t y, int64_t z){
    int64_t chunk_anchor_x = build_anchor_coord(x);
    int64_t chunk_anchor_y = build_anchor_coord(y);
    int64_t chunk_anchor_z = build_anchor_coord(z);
    //printf("building hash\n");
    uint32_t hash_table_entry =  build_chunk_hash_table_hash(chunk_anchor_x, chunk_anchor_y, chunk_anchor_z, 1586102333);
    hash_table_entry = hash_table_entry & (graph->chunk_hash_table_size - 1);
    assert(hash_table_entry < graph->chunk_hash_table_size);
    //printf("building double_hash\n");
    uint64_t double_hash = 0;
    uint64_t double_hash_val = build_chunk_hash_table_hash(chunk_anchor_x, chunk_anchor_y, chunk_anchor_z, 2734158491);
    //printf("performing double hash hashtable lookups\n");
    for(uint32_t i = 0; i < 10; i++){
        double_hash = ((uint64_t)hash_table_entry + ((i * double_hash_val))) % graph->chunk_hash_table_size;
        assert(double_hash< graph->chunk_hash_table_size);
        if(graph->chunk_hash_table[double_hash] == -1){
            return -1;
        }
        else{
            int64_t arr_index = graph->chunk_hash_table[double_hash]; 
            if(graph->chunks[arr_index].x_offset == chunk_anchor_x &&
                    graph->chunks[arr_index].y_offset == chunk_anchor_y &&
                    graph->chunks[arr_index].z_offset == chunk_anchor_z){
                return arr_index;
            }
        }
    }
    return -1;
}
int64_t voxel_graph_chunk_hash_table_request(VoxelGraph_t* graph, int64_t x, int64_t y, int64_t z){
    int64_t chunk_anchor_x = build_anchor_coord(x);
    int64_t chunk_anchor_y = build_anchor_coord(y);
    int64_t chunk_anchor_z = build_anchor_coord(z);
    //printf("building hash\n");
    uint32_t hash_table_entry =  build_chunk_hash_table_hash(chunk_anchor_x, chunk_anchor_y, chunk_anchor_z, 1586102333);
    hash_table_entry = hash_table_entry & (graph->chunk_hash_table_size - 1);
    assert(hash_table_entry < graph->chunk_hash_table_size);
    //printf("performing intial hashtable lookup\n");
    if(graph->chunk_hash_table[hash_table_entry] == -1){
        int64_t arr_index = voxel_graph_create_chunk(graph, x, y, z);
        if(arr_index == -1){
            return -1;
        }
        graph->chunk_hash_table[hash_table_entry] = arr_index;
        return arr_index;
    }
    else{
        //printf("building double_hash\n");
        uint64_t double_hash = 0;
        uint64_t double_hash_val = build_chunk_hash_table_hash(chunk_anchor_x, chunk_anchor_y, chunk_anchor_z, 2734158491);
        //printf("performing double hash hashtable lookups\n");
        for(uint32_t i = 0; i < 10; i++){
            double_hash = ((uint64_t)hash_table_entry + ((i * double_hash_val))) % graph->chunk_hash_table_size;
            if(graph->chunk_hash_table[double_hash] == -1){
                int64_t arr_index = voxel_graph_create_chunk(graph, x, y, z);
                if(arr_index == -1){
                    return -1;
                }
                graph->chunk_hash_table[double_hash] = arr_index;
                return arr_index;
            }
            else{
                int64_t arr_index = graph->chunk_hash_table[double_hash]; 
                if(graph->chunks[arr_index].x_offset == chunk_anchor_x &&
                        graph->chunks[arr_index].y_offset == chunk_anchor_y &&
                        graph->chunks[arr_index].z_offset == chunk_anchor_z){
                    return arr_index;
                }
            }
        }
        
    return -1;
    }
}
int64_t voxel_graph_create_chunk(VoxelGraph_t* graph, int64_t x, int64_t y, int64_t z){
    assert(graph != NULL);
    if(graph == NULL){
        return -1;
    }
    if(graph->current_chunk_index >= graph->chunk_amount){
        return -1;
    }
    int64_t return_index = graph->current_chunk_index;
    graph->chunks[return_index] = chunk_init();
    graph->chunks[return_index].x_offset = build_anchor_coord(x);
    graph->chunks[return_index].y_offset = build_anchor_coord(y);
    graph->chunks[return_index].z_offset = build_anchor_coord(z);
    graph->current_chunk_index++;
    return return_index;
}

void voxel_graph_free(VoxelGraph_t** graph){
    free((*graph)->chunks);
    free((*graph)->chunk_hash_table);
    free(*graph);
}






