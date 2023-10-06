/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <MaterialXFormat/XmlIo.h>

#include "material.h"
#include "node_parser.h"

#include "BKE_main.h"
#include "BLI_path_util.h"
#include "BLI_string.h"

#include "DEG_depsgraph.hh"
#include "DEG_depsgraph_query.hh"

#include "DNA_material_types.h"
#include "DNA_text_types.h"

#include "NOD_shader.h"

namespace blender::nodes::materialx {

class DefaultMaterialNodeParser : public NodeParser {
 public:
  using NodeParser::NodeParser;

  NodeItem compute() override
  {
    NodeItem surface = create_node(
        "standard_surface",
        NodeItem::Type::SurfaceShader,
        {{"base_color", val(MaterialX::Color3(material_->r, material_->g, material_->b))},
         {"diffuse_roughness", val(material_->roughness)}});

    if (material_->metallic > 0.0f) {
      surface.set_input("metalness", val(material_->metallic));
    }
    if (material_->spec) {
      surface.set_input("specular", val(material_->spec));
      surface.set_input("specular_color", val(material_->spec));
      surface.set_input("specular_roughness", val(material_->roughness));
    }

    NodeItem res = create_node(
        "surfacematerial", NodeItem::Type::Material, {{"surfaceshader", surface}});
    res.node->setName("Material_Default");
    return res;
  }

  NodeItem compute_error()
  {
    NodeItem surface = create_node("standard_surface",
                                   NodeItem::Type::SurfaceShader,
                                   {{"base_color", val(MaterialX::Color3(1.0f, 0.0f, 1.0f))}});
    NodeItem res = create_node(
        "surfacematerial", NodeItem::Type::Material, {{"surfaceshader", surface}});
    res.node->setName("Material_Error");
    return res;
  }
};

MaterialX::DocumentPtr export_to_materialx(Depsgraph *depsgraph,
                                           Material *material,
                                           ExportImageFunction export_image_fn)
{
  CLOG_INFO(LOG_MATERIALX_SHADER, 0, "Material: %s", material->id.name);

  MaterialX::DocumentPtr doc = MaterialX::createDocument();
  bool script_found = false;
  if (material->use_nodes) {
    material->nodetree->ensure_topology_cache();
    Main *bmain = DEG_get_bmain(depsgraph);
    /* Look for first script node with assigned file.
     * If it's found, we load content and ignore everything other. */
    LISTBASE_FOREACH (bNode *, node, &material->nodetree->nodes) {
      if (node->typeinfo->type == SH_NODE_SCRIPT) {
        NodeShaderScript *script = static_cast<NodeShaderScript *>(node->storage);
        if (script->mode == NODE_SCRIPT_EXTERNAL && script->filepath[0] != '\0') {
          script_found = true;
          char filepath[FILE_MAX];
          STRNCPY(filepath, script->filepath);
          BLI_path_abs(filepath, BKE_main_blendfile_path(bmain));
          try {
            MaterialX::readFromXmlFile(doc, filepath);
          }
          catch (MaterialX::ExceptionParseError) {
            CLOG_WARN(LOG_MATERIALX_SHADER,
                      "Material: %s, Node: %s: parsing error",
                      material->id.name,
                      node->name);
          }
          catch (MaterialX::ExceptionFileMissing) {
            CLOG_WARN(LOG_MATERIALX_SHADER,
                      "Material: %s, Node: %s: file not found",
                      material->id.name,
                      node->name);
          }
          break;
        }
        else if (script->mode == NODE_SCRIPT_INTERNAL) {
          Text *text = reinterpret_cast<Text *>(node->id);
          if (text) {
            script_found = true;
            std::string text_content;
            LISTBASE_FOREACH (TextLine *, line, &text->lines) {
              text_content.append(line->line);
            }
            try {
              MaterialX::readFromXmlString(doc, text_content);
            }
            catch (MaterialX::ExceptionParseError) {
              CLOG_WARN(LOG_MATERIALX_SHADER,
                        "Material: %s, Node: %s: parsing error",
                        material->id.name,
                        node->name);
            }
            break;
          }
        }
        else {
          BLI_assert_unreachable();
        }
      }
    }

    bNode *output_node = ntreeShaderOutputNode(material->nodetree, SHD_OUTPUT_ALL);
    if (output_node && !script_found) {
      NodeParserData data = {doc.get(),
                             depsgraph,
                             material,
                             NodeItem::Type::Material,
                             nullptr,
                             NodeItem(doc.get()),
                             export_image_fn};
      output_node->typeinfo->materialx_fn(&data, output_node, nullptr);
    }
    else {
      DefaultMaterialNodeParser(doc.get(),
                                depsgraph,
                                material,
                                nullptr,
                                nullptr,
                                NodeItem::Type::Material,
                                nullptr,
                                export_image_fn)
          .compute_error();
    }
  }
  else {
    DefaultMaterialNodeParser(doc.get(),
                              depsgraph,
                              material,
                              nullptr,
                              nullptr,
                              NodeItem::Type::Material,
                              nullptr,
                              export_image_fn)
        .compute();
  }

  CLOG_INFO(LOG_MATERIALX_SHADER,
            1,
            "Material: %s\n%s",
            material->id.name,
            MaterialX::writeToXmlString(doc).c_str());
  return doc;
}

}  // namespace blender::nodes::materialx
