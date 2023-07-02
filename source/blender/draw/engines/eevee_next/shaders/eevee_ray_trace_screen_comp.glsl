
void main()
{
  uint tile_id = gl_WorkGroupID.y * gl_WorkGroupSize.x + gl_WorkGroupID.x;
  /* The dispatch may bigger than necessary. */
  if (dispatch_buf.tile_count != 0u && tile_id >= dispatch_buf.tile_count) {
    return;
  }

  uvec2 tile = unpackUvec2x16(tiles_buf[tile_id]);
  ivec2 texel = ivec2(tile * gl_WorkGroupSize.xy + gl_LocalInvocationID.xy);
  ivec2 texel_fullres = texel * raytrace_buffer_buf.res_scale + raytrace_buffer_buf.res_bias;

  ssray_debug_pixel = texel_fullres;

  bool valid_texel = in_texture_range(texel_fullres, depth_tx);
  float depth = (!valid_texel) ? 0.0 : texelFetch(depth_tx, texel_fullres, 0).r;
  vec2 uv = vec2(texel_fullres) / vec2(textureSize(depth_tx, 0).xy);

  vec4 ray_data = imageLoad(inout_ray_data, texel);
  float pdf = ray_data.w;

  Ray ray;
  ray.origin = get_world_space_from_depth(uv, depth);
  ray.direction = ray_data.xyz;
  /* Extend the ray to cover the whole view. */

  vec3 radiance = vec3(0.0);
  if (pdf > 0.0) {
    vec2 noise_offset = sampling_rng_2D_get(sampling_buf, SAMPLING_RAYTRACE_W);
    float rand_trace = interlieved_gradient_noise(vec2(texel), 5.0, noise_offset.x);
    float rand_probe = interlieved_gradient_noise(vec2(texel), 7.0, noise_offset.y);

    const bool discard_backface = DO_REFLECTION;
    const bool allow_self_intersection = DO_REFRACTION;
    /* TODO(fclem): Take IOR into account in the roughness LOD bias. */
    /* TODO(fclem): pdf to roughness mapping is a crude approximation. Find something better. */
    float roughness = saturate(sample_pdf_uniform_hemisphere() / pdf);
    bool hit = false;

    ray.direction *= 1e6;
    hit = raytrace_screen(raytrace_buf,
                          hiz_buf,
                          hiz_tx,
                          rand_trace,
                          roughness,
                          discard_backface,
                          allow_self_intersection,
                          ray);

    if (hit) {
      /* Evaluate radiance at hitpoint. */
      // vec2 hit_uv = get_uvs_from_view(ray.origin + ray.direction);

      // radiance = textureLod(radiance_tx, hit_uv, 0.0).rgb;

      /* Transmit twice if thickness is set and ray is longer than thickness. */
      // if (thickness > 0.0 && length(ray_data.xyz) > thickness) {
      //   ray_radiance.rgb *= color;
      // }
    }
    else {
      radiance = lightprobe_cubemap_eval(ray.origin, ray.direction, roughness, rand_probe);
    }
  }

  /* Limit to the smallest non-0 value that the format can encode.
   * Strangely it does not correspond to the IEEE spec. */
  float inv_pdf = (pdf == 0.0) ? 0.0 : max(6e-8, 1.0 / pdf);
  /* Store inverse pdf to speedup denoising. */
  imageStore(inout_ray_data, texel, vec4(ray.direction, inv_pdf));
  imageStore(out_ray_radiance, texel, vec4(radiance, 0.0));
}
