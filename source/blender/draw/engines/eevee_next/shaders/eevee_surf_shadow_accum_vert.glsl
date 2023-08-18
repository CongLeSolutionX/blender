

/**
 * Virtual Shadow map accumulation storage.
 *
 * On Apple silicon, we can use a two-pass method to store generate shadow depth data.
 * Firstly, eevee_surf_shadow_frag is used to generate depth information which is stored
 * on tile for the closest fragment.
 *
 * This second pass then runs, writing out only the highest-level final pixel to memory,
 * avoiding the requirement for atomic texture operations.
 *
 * Output shadow atlas page indirection is calculated in the vertex shader, which generates
 * a lattice of quads covering the virtual regions which are to be updated.
 **/

#pragma BLENDER_REQUIRE(eevee_shadow_tilemap_lib.glsl)

void main()
{
  /** We create a lattice of quads covering all possible update regions.
   * We then discard regions where no updates are happening. */

  /* SHADOW_TILEMAP_RES*SHADOW_TILEMAP_RES quads per view. */
  const int quads_per_layer = (SHADOW_TILEMAP_RES * SHADOW_TILEMAP_RES);
  int shadow_update_view_id = gl_InstanceID / quads_per_layer;
  int shadow_quad_id = gl_InstanceID % quads_per_layer;
  ivec2 tile_co = ivec2(shadow_quad_id % SHADOW_TILEMAP_RES, shadow_quad_id / SHADOW_TILEMAP_RES);

  /* Determine update view and whether view is being updated. */
  gpu_Layer = shadow_update_view_id;
  gpu_ViewportIndex = int(viewport_index_buf[shadow_update_view_id]);

  /* Determine whether quad update is needed. */
  int render_page_index = shadow_render_page_index_get(shadow_update_view_id, tile_co);
  uint page_packed = render_map_buf[render_page_index];

  /* Tile not to be updated.*/
  if (page_packed == 0xFFFFFFFFu) {
    /* Discard against common update layer/viewport using degenerate primitive.
     * NaN provides fastest geom discard. */
    gpu_Layer = 0;
    gpu_ViewportIndex = 0;
    gl_Position = vec4(0.0) / vec4(0.0);
    out_texel_xy = vec2(-1000.0);
    out_page_z = 0;
    return;
  }

  /* Generate Quad. TODO(mpw_apple_gpusw/fclem): Find a nicer way of calcuating vert coords. */
  int v = gl_VertexID % 6;
  vec2 pos;
  switch (v) {
    case 1:
      pos.x = 0.0;
      pos.y = 0.0;
      break;
    case 2:
    case 4:
      pos.x = 256.0;
      pos.y = 0.0;
      break;

    case 0:
    case 3:
      pos.x = 0.0;
      pos.y = 256.0;
      break;
    case 5:
      pos.x = 256.0;
      pos.y = 256.0;
      break;
  };

  /** Interpolate output texel  */
  /* Using bitwise ops is way faster than integer ops. */
  const int page_shift = SHADOW_PAGE_LOD;
  const int page_mask = ~(0xFFFFFFFF << SHADOW_PAGE_LOD);

  uint3 page = uint3(shadow_page_unpack(page_packed));
  out_texel_xy = (vec2(page.xy) * vec2(256.0)) + vec2(pos);
  out_page_z = float(page.z);

  /** Output quad position*/
  /* Offset quad coordinates in screen space. */
  pos += vec2(tile_co) * 256.0;

  /* Transform position into NDC accounting for target viewport size. */
  const float viewport_dim_inv = 1.0f / float((SHADOW_TILEMAP_RES * SHADOW_PAGE_RES) >>
                                              (SHADOW_TILEMAP_LOD -
                                               clamp(gpu_ViewportIndex, 0, SHADOW_TILEMAP_LOD)));
  pos = pos * vec2(viewport_dim_inv * 2.0) - vec2(1.0);

  /* NOTE: To avoid redundant writes, we still enable the depth test and configure the accumulation
   * pass quad depth such that only the fragments updated during the surface depth pass will run.
   *
   * We use `DRW_STATE_DEPTH_GREATER_EQUAL` such that we only run the fragment shader if the
   * existing depth value is smaller than `depth_update_treshold`.  */
  const float depth_update_treshold = 0.99;
  gl_Position = vec4(pos.x, pos.y, depth_update_treshold, 1.0);
}
