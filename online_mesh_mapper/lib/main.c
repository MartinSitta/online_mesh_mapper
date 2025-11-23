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

    voxel_graph_insert(graph, 1, 1, 2);
    voxel_graph_insert(graph, 1, 2, 1);
    voxel_graph_insert(graph, 2, 1, 1);
    voxel_graph_delete(graph, 1, 1, 1);

    uint16_t test_coord_center = build_vertex_coords(1, 1, 1);
    uint16_t test_coord_above = build_vertex_coords(1, 1, 2);
    uint16_t test_coord_left = build_vertex_coords(1, 2, 1);
    uint16_t test_coord_ahead = build_vertex_coords(2, 1, 1);

    int64_t test_index_center = chunk_node_lookup(&graph->chunks[0], test_coord_center);
    int64_t test_index_above = chunk_node_lookup(&graph->chunks[0], test_coord_above);
    int64_t test_index_left = chunk_node_lookup(&graph->chunks[0], test_coord_left);
    int64_t test_index_ahead = chunk_node_lookup(&graph->chunks[0], test_coord_ahead);

    assert(graph->chunks[0].nodes[test_index_center].coord_and_mesh_info.buf[2] == 1);
    assert(!vertex_get_down_bit(&graph->chunks[0].nodes[test_index_above].coord_and_mesh_info));
    assert(!vertex_get_right_bit(&graph->chunks[0].nodes[test_index_left].coord_and_mesh_info));
    assert(!vertex_get_back_bit(&graph->chunks[0].nodes[test_index_ahead].coord_and_mesh_info));
    
    voxel_graph_insert(graph, 1, 1, 1);

    assert(vertex_get_down_bit(&graph->chunks[0].nodes[test_index_above].coord_and_mesh_info));
    assert(vertex_get_right_bit(&graph->chunks[0].nodes[test_index_left].coord_and_mesh_info));
    assert(vertex_get_back_bit(&graph->chunks[0].nodes[test_index_ahead].coord_and_mesh_info));
    
    assert(vertex_get_up_bit(&graph->chunks[0].nodes[test_index_center].coord_and_mesh_info));
    assert(vertex_get_left_bit(&graph->chunks[0].nodes[test_index_center].coord_and_mesh_info));
    assert(vertex_get_foward_bit(&graph->chunks[0].nodes[test_index_center].coord_and_mesh_info));
    assert(!vertex_get_dead_bit(&graph->chunks[0].nodes[test_index_center].coord_and_mesh_info));

    voxel_graph_free(&graph);
    return 0;
}
