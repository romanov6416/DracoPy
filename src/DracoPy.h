#include<vector>
#include<cstddef>
#include <string>
#include <sstream>
#include <iostream>

#include "draco/compression/decode.h"
#include "draco/compression/encode.h"
#include "draco/core/encoder_buffer.h"
#include "draco/core/vector_d.h"
#include "draco/mesh/triangle_soup_mesh_builder.h"
#include "draco/point_cloud/point_cloud_builder.h"

namespace DracoFunctions {
    using namespace draco;

  enum decoding_status { successful, not_draco_encoded, no_position_attribute, failed_during_decoding };
  enum encoding_status { successful_encoding, failed_during_encoding };

  struct PointAttributeObject {
    std::unordered_map<int, int> d;
  };

  struct PointCloudObject {
    std::vector<float> points;

    // Encoding options stored in metadata
    bool encoding_options_set;
    int quantization_bits;
    double quantization_range;
    std::vector<double> quantization_origin;

    decoding_status decode_status;

    // not parsed metadata
    std::string binary_metadata;
  };

  struct MeshObject : PointCloudObject {
    std::vector<float> normals;
    std::vector<unsigned int> faces;
  };

  struct EncodedObject {
    std::vector<unsigned char> buffer;
    encoding_status encode_status;
  };

  class MetadataReader {
  public:
    MetadataReader(const std::string& s): stream(s) {}

    uint32_t read_uint32() {
      uint32_t value;
      stream.read(reinterpret_cast<char*>(&value), sizeof(uint32_t));
      return value;
    }

    std::string read_bytes(const uint32_t& len) {
      std::string value;
      value.resize(len);
      stream.read(reinterpret_cast<char*>(value.data()), len);
      return value;
    }

    std::string read_bytes_with_size() {
      const auto str_len = read_uint32();
      return read_bytes(str_len);
    }

    std::vector<uint8_t> read_bytes_to_vec() {
      const auto vec_len = read_uint32();
      std::vector<uint8_t> value;
      value.resize(vec_len);
      stream.read(reinterpret_cast<char*>(value.data()), vec_len);
      return value;
    }

  private:
    std::stringstream stream;
  };

  class MetadataWriter {
  public:
    void write_uint32(const uint32_t& value) {
      stream.write(reinterpret_cast<const char*>(&value), sizeof(uint32_t));
    }

    template <class T>
    void write_uint(const T& value) {
      write_uint32(static_cast<const uint32_t&>(value));
    }

    template <class StdContainer>
    void write_bytes(const StdContainer& value) {
      // write_uint(static_cast<uint32_t>(value.size()));
      stream.write(reinterpret_cast<const char*>(value.data()), value.size());
    }

    void write_bytes_from_str(const std::string& value) {
        write_bytes(value);
    }

    void write_bytes_from_vec(const std::vector<uint8_t>& value) {
        write_bytes(value);
    }

    void write_bytes_from_ptr(const void* ptr, uint32_t len) {
        stream.write(reinterpret_cast<const char*>(ptr), len);
    }

    std::string get() { return stream.str(); }

  private:
    std::stringstream stream;
  };

  // std::vector<const PointAttribute*> 

  std::string encode_metadata(const Mesh& mesh) {
    if (!mesh.GetMetadata())
      return {};
    const auto& geometry_metadata = *mesh.GetMetadata();
    std::vector<const Metadata*> to_parse_metadata =
            { {static_cast<const Metadata*>(&geometry_metadata)} };
    MetadataWriter writer;
    // consider attribute metadatas
    const auto& attribute_metadatas = geometry_metadata.attribute_metadatas();
    std::vector<const PointAttribute*> vertices_attributes, faces_attributes;
    writer.write_uint(attribute_metadatas.size());
    for (const auto& attribute_metadata: attribute_metadatas) {
      const auto unique_id = attribute_metadata->att_unique_id();
      uint32_t attribute_type = 0;
      uint32_t element_size = 0;
      if (const auto* attribute = mesh.GetAttributeByUniqueId(unique_id)) {
        const auto attr_id = mesh.GetAttributeIdByUniqueId(unique_id);
        const auto draco_attr_type = mesh.GetAttributeElementType(attr_id);
        std::cout << unique_id << ' ' << attr_id << ' ' << draco_attr_type << std::endl;
        switch (draco_attr_type) {
          case MeshAttributeElementType::MESH_VERTEX_ATTRIBUTE:
            attribute_type = 1;
            vertices_attributes.push_back(attribute);
            break;
          case MeshAttributeElementType::MESH_FACE_ATTRIBUTE:
            attribute_type = 2;
            faces_attributes.push_back(attribute);
            break;
          case MeshAttributeElementType::MESH_CORNER_ATTRIBUTE:
            attribute_type = 3;
            vertices_attributes.push_back(attribute);
            break;
          default:
            // unknown draco element type
            attribute_type = 0;
            break;
        }
        // std::cout << "attribute_type: " << attribute_type << std::endl;
        if (attribute_type != 0)
          element_size = static_cast<uint32_t>(attribute->byte_stride());
      }
      writer.write_uint(unique_id);
      writer.write_uint(attribute_type);
      writer.write_uint(element_size);
      to_parse_metadata.push_back(static_cast<const Metadata*>(attribute_metadata.get()));
    }
    // consider attribute metadatas geometry data
    for (PointIndex v(0); v < mesh.num_points(); ++v)
      for (const auto* vertexAttribute: vertices_attributes) {
        const auto* value = vertexAttribute->GetAddress(vertexAttribute->mapped_index(v));
        const auto len = static_cast<uint32_t>(vertexAttribute->byte_stride());
        writer.write_bytes_from_ptr(value, len);
      }
    for (FaceIndex f(0); f < mesh.num_faces(); ++f) {
      const auto face = mesh.face(f);
      for (const auto* faceAttribute: faces_attributes) {
        const auto* value = faceAttribute->GetAddress(faceAttribute->mapped_index(face[0]));
        const auto len = static_cast<uint32_t>(faceAttribute->byte_stride());
        writer.write_bytes_from_ptr(value, len);
      }
    }
    // encode metadatas level by level
    for (std::vector<const Metadata*> to_parse_metadatas_next;
            !to_parse_metadata.empty();
            to_parse_metadata = std::move(to_parse_metadatas_next)) {
      for (const auto* draco_metadata: to_parse_metadata) {
        // encode entries
        const auto& entries = draco_metadata->entries();
        writer.write_uint(entries.size());
        for (const auto& [name, value]: draco_metadata->entries()) {
            writer.write_uint32(name.size());
            writer.write_bytes(name);
            writer.write_uint32(value.data().size());
            writer.write_bytes(value.data());
        }
        // consider sub metadatas
        const auto& sub_metadatas = draco_metadata->sub_metadatas();
        writer.write_uint(sub_metadatas.size());
        for (const auto& [name, draco_sub_metadata]: sub_metadatas) {
            writer.write_uint(name.size());
            writer.write_bytes(name);
            to_parse_metadatas_next.push_back(draco_sub_metadata.get());
        }
      }
    }
    return writer.get();
  }

  GeometryMetadata decode_metadata(const std::string& s) {
    MetadataReader reader(s);
    GeometryMetadata geometry_metadata;
    std::vector<Metadata*> to_parse_metadata = {
            {static_cast<Metadata*>(&geometry_metadata)} };
    // consider attribute metadatas
    const auto attrbite_metadatas_len = reader.read_uint32();
    for (uint32_t i = 0; i < attrbite_metadatas_len; ++i) {
        auto attribute_metadata = std::make_unique<AttributeMetadata>();
        const auto unique_id = reader.read_uint32();
        attribute_metadata->set_att_unique_id(unique_id);
        to_parse_metadata.push_back(attribute_metadata.get());
        geometry_metadata.AddAttributeMetadata(std::move(attribute_metadata));
    }
    // parse metadatas level by level
    for (std::vector<Metadata*> to_parse_metadatas_next;
            !to_parse_metadata.empty();
            to_parse_metadata = std::move(to_parse_metadatas_next)) {
         for (auto* metadata: to_parse_metadata) {
            // parse entries
            const auto entries_len = reader.read_uint32();
            for (uint32_t i = 0; i < entries_len; ++i) {
                std::string name = reader.read_bytes_with_size();
                std::vector<uint8_t> value = reader.read_bytes_to_vec();
                metadata->AddEntryBinary(name, value);
            }
            // consider sub metadatas
            const auto sub_metadatas_len = reader.read_uint32();
            for (uint32_t i = 0; i < sub_metadatas_len; ++i) {
                auto sub_metadata = std::make_unique<Metadata>();
                to_parse_metadatas_next.push_back(sub_metadata.get());
                std::string name = reader.read_bytes_with_size();
                metadata->AddSubMetadata(name, std::move(sub_metadata));
            }
         }
    }
    return geometry_metadata;

  }

  MeshObject decode_buffer(const char *buffer, std::size_t buffer_len) {
    MeshObject meshObject;
    DecoderBuffer decoderBuffer;
    decoderBuffer.Init(buffer, buffer_len);
    Decoder decoder;
    auto statusor = decoder.DecodeMeshFromBuffer(&decoderBuffer);
    if (!statusor.ok()) {
      std::string status_string = statusor.status().error_msg_string();
      if (status_string.compare("Not a Draco file.") || status_string.compare("Failed to parse Draco header.")) {
        meshObject.decode_status = not_draco_encoded;
      }
      else {
        meshObject.decode_status = failed_during_decoding;
      }
      return meshObject;
    }
    std::unique_ptr<Mesh> in_mesh = std::move(statusor).value();
    Mesh *mesh = in_mesh.get();
    const int pos_att_id = mesh->GetNamedAttributeId(GeometryAttribute::POSITION);
    if (pos_att_id < 0) {
      meshObject.decode_status = no_position_attribute;
      return meshObject;
    }
    meshObject.points.reserve(3 * mesh->num_points());
    meshObject.faces.reserve(3 * mesh->num_faces());
    const auto *const pos_att = mesh->attribute(pos_att_id);
    std::array<float, 3> pos_val;
    for (PointIndex v(0); v < mesh->num_points(); ++v) {
      if (!pos_att->ConvertValue<float, 3>(pos_att->mapped_index(v), &pos_val[0])) {
        meshObject.decode_status = no_position_attribute;
        return meshObject;
      }
      meshObject.points.push_back(pos_val[0]);
      meshObject.points.push_back(pos_val[1]);
      meshObject.points.push_back(pos_val[2]);
    }
    for (FaceIndex i(0); i < mesh->num_faces(); ++i) {
      const auto &f = mesh->face(i);
      meshObject.faces.push_back(*(reinterpret_cast<const uint32_t *>(&(f[0]))));
      meshObject.faces.push_back(*(reinterpret_cast<const uint32_t *>(&(f[1]))));
      meshObject.faces.push_back(*(reinterpret_cast<const uint32_t *>(&(f[2]))));
    }
    const GeometryMetadata *metadata = mesh->GetMetadata();
    meshObject.encoding_options_set = false;
    if (metadata) {
      metadata->GetEntryInt("quantization_bits", &(meshObject.quantization_bits));
      if (metadata->GetEntryDouble("quantization_range", &(meshObject.quantization_range)) &&
          metadata->GetEntryDoubleArray("quantization_origin", &(meshObject.quantization_origin))) {
          meshObject.encoding_options_set = true;
      }
      meshObject.binary_metadata = encode_metadata(*mesh);
    }
    meshObject.decode_status = successful;
    return meshObject;
  }

  PointCloudObject decode_buffer_to_point_cloud(const char *buffer, std::size_t buffer_len) {
    PointCloudObject pointCloudObject;
    DecoderBuffer decoderBuffer;
    decoderBuffer.Init(buffer, buffer_len);
    Decoder decoder;
    auto statusor = decoder.DecodePointCloudFromBuffer(&decoderBuffer);
    if (!statusor.ok()) {
      std::string status_string = statusor.status().error_msg_string();
      if (status_string.compare("Not a Draco file.") || status_string.compare("Failed to parse Draco header.")) {
        pointCloudObject.decode_status = not_draco_encoded;
      }
      else {
        pointCloudObject.decode_status = failed_during_decoding;
      }
      return pointCloudObject;
    }
    std::unique_ptr<PointCloud> in_point_cloud = std::move(statusor).value();
    PointCloud *point_cloud = in_point_cloud.get();
    const int pos_att_id = point_cloud->GetNamedAttributeId(GeometryAttribute::POSITION);
    if (pos_att_id < 0) {
      pointCloudObject.decode_status = no_position_attribute;
      return pointCloudObject;
    }
    pointCloudObject.points.reserve(3 * point_cloud->num_points());
    const auto *const pos_att = point_cloud->attribute(pos_att_id);
    std::array<float, 3> pos_val;
    for (PointIndex v(0); v < point_cloud->num_points(); ++v) {
      if (!pos_att->ConvertValue<float, 3>(pos_att->mapped_index(v), &pos_val[0])) {
        pointCloudObject.decode_status = no_position_attribute;
        return pointCloudObject;
      }
      pointCloudObject.points.push_back(pos_val[0]);
      pointCloudObject.points.push_back(pos_val[1]);
      pointCloudObject.points.push_back(pos_val[2]);
    }
    const GeometryMetadata *metadata = point_cloud->GetMetadata();
    pointCloudObject.encoding_options_set = false;
    if (metadata) {
      metadata->GetEntryInt("quantization_bits", &(pointCloudObject.quantization_bits));
      if (metadata->GetEntryDouble("quantization_range", &(pointCloudObject.quantization_range)) &&
        metadata->GetEntryDoubleArray("quantization_origin", &(pointCloudObject.quantization_origin))) {
        pointCloudObject.encoding_options_set = true;
      }
      pointCloudObject.binary_metadata = ""; // PointCloud metadata is not supported
    }
    pointCloudObject.decode_status = successful;
    return pointCloudObject;
  }

  void setup_encoder_and_metadata(PointCloud *point_cloud_or_mesh, Encoder &encoder, int compression_level, int quantization_bits, float quantization_range, const float *quantization_origin, bool create_metadata) {
    int speed = 10 - compression_level;
    encoder.SetSpeedOptions(speed, speed);
    std::unique_ptr<GeometryMetadata> metadata = std::unique_ptr<GeometryMetadata>(new GeometryMetadata());
    if (quantization_origin == NULL || quantization_range == -1) {
      encoder.SetAttributeQuantization(GeometryAttribute::POSITION, quantization_bits);
    }
    else {
      encoder.SetAttributeExplicitQuantization(GeometryAttribute::POSITION, quantization_bits, 3, quantization_origin, quantization_range);
      if (create_metadata) {
        metadata->AddEntryDouble("quantization_range", quantization_range);
        std::vector<double> quantization_origin_vec;
        for (int i = 0; i < 3; i++) {
          quantization_origin_vec.push_back(quantization_origin[i]);
        }
        metadata->AddEntryDoubleArray("quantization_origin", quantization_origin_vec);
      }
    }
    if (create_metadata) {
      metadata->AddEntryInt("quantization_bits", quantization_bits);
      point_cloud_or_mesh->AddMetadata(std::move(metadata));
    }
  }

  EncodedObject encode_mesh(const std::vector<float> &points, const
  std::vector<unsigned int> &faces, const std::string& binary_metadata,
      int quantization_bits, int compression_level, float quantization_range, const float *quantization_origin, bool create_metadata) {
    TriangleSoupMeshBuilder mb;
    mb.Start(faces.size());
    const int pos_att_id =
      mb.AddAttribute(GeometryAttribute::POSITION, 3, DataType::DT_FLOAT32);

    for (std::size_t i = 0; i <= faces.size() - 3; i += 3) {
      auto point1Index = faces[i]*3;
      auto point2Index = faces[i+1]*3;
      auto point3Index = faces[i+2]*3;
      mb.SetAttributeValuesForFace(pos_att_id, FaceIndex(i), Vector3f(points[point1Index], points[point1Index+1], points[point1Index+2]).data(), Vector3f(points[point2Index], points[point2Index+1], points[point2Index+2]).data(), Vector3f(points[point3Index], points[point3Index+1], points[point3Index+2]).data());
    }

    std::unique_ptr<Mesh> ptr_mesh = mb.Finalize();
    Mesh *mesh = ptr_mesh.get();
    Encoder encoder;
    setup_encoder_and_metadata(mesh, encoder, compression_level, quantization_bits, quantization_range, quantization_origin, create_metadata);
    EncoderBuffer buffer;
    const Status status = encoder.EncodeMeshToBuffer(*mesh, &buffer);
    EncodedObject encodedMeshObject;
    encodedMeshObject.buffer = *((std::vector<unsigned char> *)buffer.buffer());
    if (status.ok()) {
      encodedMeshObject.encode_status = successful_encoding;
    } else {
      std::cout << "Draco encoding error: " << status.error_msg_string() << std::endl;
      encodedMeshObject.encode_status = failed_during_encoding;
    }
    return encodedMeshObject;
  }

  EncodedObject encode_point_cloud(const std::vector<float> &points,
   const std::string& binary_metadata, int quantization_bits,
      int compression_level, float quantization_range, const float *quantization_origin, bool create_metadata) {
    int num_points = points.size() / 3;
    PointCloudBuilder pcb;
    pcb.Start(num_points);
    const int pos_att_id =
      pcb.AddAttribute(GeometryAttribute::POSITION, 3, DataType::DT_FLOAT32);

    for (PointIndex i(0); i < num_points; i++) {
      pcb.SetAttributeValueForPoint(pos_att_id, i, points.data() + 3 * i.value());
    }

    std::unique_ptr<PointCloud> ptr_point_cloud = pcb.Finalize(true);
    PointCloud *point_cloud = ptr_point_cloud.get();
    Encoder encoder;
    setup_encoder_and_metadata(point_cloud, encoder, compression_level, quantization_bits, quantization_range, quantization_origin, create_metadata);
    EncoderBuffer buffer;
    const Status status = encoder.EncodePointCloudToBuffer(*point_cloud, &buffer);
    EncodedObject encodedPointCloudObject;
    encodedPointCloudObject.buffer = *((std::vector<unsigned char> *)buffer.buffer());
    if (status.ok()) {
      encodedPointCloudObject.encode_status = successful_encoding;
    } else {
      std::cout << "Draco encoding error: " << status.error_msg_string() << std::endl;
      encodedPointCloudObject.encode_status = failed_during_encoding;
    }
    return encodedPointCloudObject;
  }

  // void test_encoder() {
  //   Mesh mesh;
  //   // auto attribute = std::make_unique<PointAttribute>();
  //   // attribute->set_unique_id(0);
  //   GeometryAttribute attribute;
  //   attribute.Init(GeometryAttribute::GENERIC, nullptr, 0, DT_INT8, false, 0, 0);
  //   int attribute_id = mesh.AddAttribute(attribute, true, 3);
  //   if (attribute_id < 0) {
  //     std::cout << "crash" << std::endl;
  //     throw "fail";
  //   }

  //   mesh.SetAttributeElementType(attribute_id, MESH_VERTEX_ATTRIBUTE);

  //   Encoder encoder;
  //   EncoderBuffer buffer;
  //   const Status status = encoder.EncodeMeshToBuffer(mesh, &buffer);

  //   DecoderBuffer decoderBuffer;
  //   decoderBuffer.Init(buffer.data(), buffer.size());
  //   Decoder decoder;
  //   auto statusor = decoder.DecodeMeshFromBuffer(&decoderBuffer);
  //   std::unique_ptr<Mesh> in_mesh = std::move(statusor).value();
  //   Mesh* new_mesh = in_mesh.get();

  //   std::cout << "get new attr" << std::endl;
  //   auto* new_attribute = new_mesh->GetAttributeByUniqueId(0);
  //   std::cout << "get new attr id" << std::endl;
  //   auto new_attriute_id = new_mesh->GetAttributeIdByUniqueId(0);
  //   std::cout << "get element type" << std::endl;
  //   const auto element_type = new_mesh->GetAttributeElementType(attribute_id);
  //   std::cout << "check" << std::endl;
  //   std::cout << element_type << " != " << MESH_VERTEX_ATTRIBUTE << std::endl;
  // }
}
