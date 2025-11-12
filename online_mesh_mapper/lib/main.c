#include <stdio.h>
#include "Chunk.h"
#include "VoxelGraph.h"
#include "HashUtils.h"

int main(){
    printf("%ld\n", sizeof(GraphNode_t));
    printf("%ld\n", sizeof(Chunk_t));
    VoxelGraph_t* graph = voxel_graph_init(4096);
    printf("graph chunk_array_size is %ld \n",(long) graph->chunk_amount);
    
    printf("graph chunk_hash_table_size is %ld \n", (long) graph->chunk_hash_table_size);
    printf("size of graph is %ld\n", sizeof(*graph) + sizeof(*(graph->chunks)) * graph->chunk_amount + sizeof(*(graph->chunk_hash_table)) * graph->chunk_hash_table_size);
    printf("hash-test 1: %d\n", build_chunk_hash_table_hash(2354, 123, 4, 032540327));

    uint16_t coords = build_vertex_coords(16, 16, 16);
    printf("coords sanity check %d\n", coords);
    printf("%d\n%d\n%d\n", vertex_pick_x_coord(coords), vertex_pick_y_coord(coords), vertex_pick_z_coord(coords));

    printf("anchor coord sanity test\n");
    printf("%ld\n", build_anchor_coord(32));
    printf("%ld\n", build_anchor_coord(0));
    printf("%ld\n", build_anchor_coord(-1));

    printf("hash-test 1: %d\n", build_node_hash_table_hash(coords, 032540327));
    voxel_graph_insert(graph, 1, 1, 1);
    assert(graph->current_chunk_index == 1);
    assert(graph->chunks[0].x_offset == 0);
    assert(graph->chunks[0].y_offset == 0);
    assert(graph->chunks[0].z_offset == 0);
    assert(graph->chunks[0].current_node_index == 1);
    assert(graph->chunks[0].nodes[0].coord_and_mesh_info.vertex_coords != 0);
    voxel_graph_insert(graph, 2, 2, 2);
    assert(graph->current_chunk_index == 1);
    assert(graph->chunks[0].x_offset == 0);
    assert(graph->chunks[0].y_offset == 0);
    assert(graph->chunks[0].z_offset == 0);
    assert(graph->chunks[0].current_node_index == 2);
    assert(graph->chunks[0].nodes[1].coord_and_mesh_info.vertex_coords != 0);
    voxel_graph_insert(graph, 2, 2, 2);
    assert(graph->current_chunk_index == 1);
    assert(graph->chunks[0].x_offset == 0);
    assert(graph->chunks[0].y_offset == 0);
    assert(graph->chunks[0].z_offset == 0);
    assert(graph->chunks[0].current_node_index == 2);

    voxel_graph_insert(graph, 2, -1, 2);
    assert(graph->current_chunk_index == 2);
    assert(graph->chunks[1].x_offset == 0);
    assert(graph->chunks[1].y_offset == -32);
    assert(graph->chunks[1].z_offset == 0);
    assert(graph->chunks[1].current_node_index == 1);

    voxel_graph_insert(graph, 1, 1, 2);
    assert(graph->current_chunk_index == 2);
    assert(graph->chunks[0].x_offset == 0);
    assert(graph->chunks[0].y_offset == 0);
    assert(graph->chunks[0].z_offset == 0);
    assert(graph->chunks[0].current_node_index == 3);
    assert(graph->chunks[0].nodes[2].coord_and_mesh_info.buf[2] != 0);
    assert(graph->chunks[0].nodes[0].coord_and_mesh_info.buf[2] != 0);
    //for debugging
    voxel_graph_insert(graph, -9, -32, 12);
    voxel_graph_insert(graph,167, 82 ,-1);
    voxel_graph_free(&graph);
    return 0;
}
