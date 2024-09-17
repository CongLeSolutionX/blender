/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

void npr_input(out TextureHandle combined_color,
               out TextureHandle diffuse_color,
               out TextureHandle diffuse_direct,
               out TextureHandle diffuse_indirect,
               out TextureHandle specular_color,
               out TextureHandle specular_direct,
               out TextureHandle specular_indirect,
               out TextureHandle position,
               out TextureHandle normal)
{
#if defined(NPR_SHADER) && defined(GPU_FRAGMENT_SHADER)
  npr_input_impl(combined_color,
                 diffuse_color,
                 diffuse_direct,
                 diffuse_indirect,
                 specular_color,
                 specular_direct,
                 specular_indirect,
                 position,
                 normal);
#else
  combined_color = TEXTURE_HANDLE_DEFAULT;
  diffuse_color = TEXTURE_HANDLE_DEFAULT;
  diffuse_direct = TEXTURE_HANDLE_DEFAULT;
  diffuse_indirect = TEXTURE_HANDLE_DEFAULT;
  specular_color = TEXTURE_HANDLE_DEFAULT;
  specular_direct = TEXTURE_HANDLE_DEFAULT;
  specular_indirect = TEXTURE_HANDLE_DEFAULT;
  position = TEXTURE_HANDLE_DEFAULT;
  normal = TEXTURE_HANDLE_DEFAULT;
#endif
}

void npr_output(vec4 color, out vec4 out_color)
{
  out_color = color;
}

void npr_refraction(out TextureHandle combined_color, out TextureHandle position)
{
#if defined(NPR_SHADER) && defined(GPU_FRAGMENT_SHADER)
  npr_refraction_impl(combined_color, position);
#else
  combined_color = TEXTURE_HANDLE_DEFAULT;
  position = TEXTURE_HANDLE_DEFAULT;
#endif
}
