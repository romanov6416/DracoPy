# distutils: language = c++
from typing import Dict, List

from cpython.mem cimport PyMem_Malloc, PyMem_Free
cimport DracoPy
from math import floor


class MetadataObject:
    def __init__(self, entries: Dict[str, bytes] = None,
                 sub_metadatas: Dict[str, 'MetadataObject'] = None):
        self.entries = entries if entries else {}
        self.sub_metadatas = sub_metadatas if sub_metadatas else {}


class AttributeMetadataObject(MetadataObject):
    def __init__(self, unique_id: int,
                 entries: Dict[str, bytes] = None,
                 sub_metadatas: Dict[str, 'MetadataObject'] = None):
        super().__init__(entries, sub_metadatas)
        self.unique_id = unique_id


class GeometryMetadataObject(MetadataObject):
    def __init__(self, entries: Dict[str, bytes] = None,
                 sub_metadatas: Dict[str, 'MetadataObject'] = None,
                 attribute_metadatas: List['AttributeMetadataObject'] = None):
        super().__init__(entries, sub_metadatas)
        self.attribute_metadatas = attribute_metadatas if attribute_metadatas \
            else []


def decode_metadata(binary_metadata: bytes) -> MetadataObject:
    reader = new DracoPy.MetadataReader(binary_metadata)
    geometry_metadata = GeometryMetadataObject()
    to_parse_metadatas = [geometry_metadata]
    # consider attribute metadatas
    attribute_metadatas_len = reader.read_uint()
    for _ in range(attribute_metadatas_len):
        unique_id = reader.read_uint()
        attribute_metadata = AttributeMetadataObject(unique_id)
        geometry_metadata.attribute_metadatas.append(attribute_metadata)
        to_parse_metadatas.append(attribute_metadata)
    # parse metadatas level by level
    while to_parse_metadatas:
        to_parse_metadata_next = []
        for metadata in to_parse_metadatas:
            # parse entries
            entries_len = reader.read_uint()
            for _ in range(entries_len):
                name = reader.read_bytes()
                value = reader.read_bytes()
                metadata.entries[name] = value
            sub_metadatas_len = reader.read_uint()
            # consider sub metadatas
            for _ in range(sub_metadatas_len):
                name = reader.read_bytes()
                sub_metadata = MetadataObject()
                metadata.sub_metadatas[name] = sub_metadata
                to_parse_metadata_next.append(sub_metadata)
        to_parse_metadatas = to_parse_metadata_next
    del reader
    return geometry_metadata

def encode_metadata(geometry_metadata: GeometryMetadataObject) -> bytes:
    cdef DracoPy.MetadataWriter writer
    to_parse_metadata = [geometry_metadata]
    # consider attribute metadatas
    writer.write_uint(len(geometry_metadata.attribute_metadatas))
    for attribute_metadata in geometry_metadata.attribute_metadatas:
        writer.write_uint(attribute_metadata.unique_id)
        to_parse_metadata.append(attribute_metadata)
    # encode metadatas level by level
    while to_parse_metadata:
        to_parse_metadata_next = []
        for draco_metadata in to_parse_metadata:
            # encode entries
            writer.write_uint(len(draco_metadata.entries))
            for name, value in draco_metadata.entries.items():
                writer.write_bytes_from_str(name)
                writer.write_bytes_from_vec(value)
            # consider sub metadatas
            writer.write_uint(len(draco_metadata.sub_metadatas))
            for name, draco_sub_metadata in draco_metadata.sub_metadatas.items():
                writer.write_bytes_from_str(name)
                to_parse_metadata_next.append(draco_sub_metadata)
    return writer.get()


class DracoPointCloud(object):
    def __init__(self, data_struct):
        self.data_struct = data_struct
        if data_struct['encoding_options_set']:
            self.encoding_options = EncodingOptions(data_struct['quantization_bits'],
                data_struct['quantization_range'], data_struct['quantization_origin'])
        else:
            self.encoding_options = None
        self.metadata = decode_metadata(data_struct["binary_metadata"])

    def get_encoded_coordinate(self, value, axis):
        if self.encoding_options is not None:
            return self.encoding_options.get_encoded_coordinate(value, axis)

    def get_encoded_point(self, point):
        if self.encoding_options is not None:
            return self.encoding_options.get_encoded_point(point)

    @property
    def num_axes(self):
        return 3

    @property
    def points(self):
        return self.data_struct['points']


class DracoMesh(DracoPointCloud):
    @property
    def faces(self):
        return self.data_struct['faces']

    @property
    def normals(self):
        return self.data_struct['normals']

class EncodingOptions(object):
    def __init__(self, quantization_bits, quantization_range, quantization_origin):
        self.quantization_bits = quantization_bits
        self.quantization_range = quantization_range
        self.quantization_origin = quantization_origin
        self.inverse_alpha = quantization_range / ((2 ** quantization_bits) - 1)

    def get_encoded_coordinate(self, value, axis):
        if value < self.quantization_origin[axis] or value > (self.quantization_origin[axis] + self.quantization_range):
            raise ValueError('Specified value out of encoded range')
        difference = value - self.quantization_origin[axis]
        quantized_index = floor((difference / self.inverse_alpha) + 0.5)
        return self.quantization_origin[axis] + (quantized_index * self.inverse_alpha)

    def get_encoded_point(self, point):
        encoded_point = []
        for axis in range(self.num_axes):
            encoded_point.append(self.get_encoded_coordinate(point[axis], axis))
        return encoded_point

    @property
    def num_axes(self):
        return 3

class FileTypeException(Exception):
    pass

class EncodingFailedException(Exception):
    pass

def encode_mesh_to_buffer(points, faces, metadata: GeometryMetadataObject = None,
                          quantization_bits=14, compression_level=1,
                          quantization_range=-1, quantization_origin=None,
                          create_metadata=False):
    """
    Encode a list or numpy array of points/vertices (float) and faces (unsigned int) to a draco buffer.
    Quantization bits should be an integer between 0 and 31
    Compression level should be an integer between 0 and 10
    Quantization_range is a float representing the size of the bounding cube for the mesh.
    By default it is the range of the dimension of the input vertices with greatest range.
    Quantization_origin is the point in space where the bounding box begins. By default it is
    a point where each coordinate is the minimum of that coordinate among the input vertices.
    """
    if metadata is None:
        metadata = GeometryMetadataObject()
    cdef float* quant_origin = NULL
    try:
        num_dims = 3
        if quantization_origin is not None:
            quant_origin = <float *>PyMem_Malloc(sizeof(float) * num_dims)
            for dim in range(num_dims):
                quant_origin[dim] = quantization_origin[dim]
        binary_metadata = encode_metadata(metadata)
        encoded_mesh = DracoPy.encode_mesh(points, faces, binary_metadata,
                                           quantization_bits, compression_level,
                                           quantization_range, quant_origin,
                                           create_metadata)
        if quant_origin != NULL:
            PyMem_Free(quant_origin)
        if encoded_mesh.encode_status == DracoPy.encoding_status.successful_encoding:
            return bytes(encoded_mesh.buffer)
        elif encoded_mesh.encode_status == DracoPy.encoding_status.failed_during_encoding:
            raise EncodingFailedException('Invalid mesh')
    except EncodingFailedException:
        raise EncodingFailedException('Invalid mesh')
    except:
        if quant_origin != NULL:
            PyMem_Free(quant_origin)
        raise ValueError("Input invalid")

def encode_point_cloud_to_buffer(points, metadata: GeometryMetadataObject = None,
                                 quantization_bits=14, compression_level=1,
                                 quantization_range=-1, quantization_origin=None,
                                 create_metadata=False):
    """
    Encode a list or numpy array of points/vertices (float) to a draco buffer.
    Quantization bits should be an integer between 0 and 31
    Compression level should be an integer between 0 and 10
    Quantization_range is a float representing the size of the bounding cube for the mesh.
    By default it is the range of the dimension of the input vertices with greatest range.
    Quantization_origin is the point in space where the bounding box begins. By default it is
    a point where each coordinate is the minimum of that coordinate among the input vertices.
    """
    if metadata is None:
        metadata = GeometryMetadataObject()
    cdef float* quant_origin = NULL
    try:
        num_dims = 3
        if quantization_origin is not None:
            quant_origin = <float *>PyMem_Malloc(sizeof(float) * num_dims)
            for dim in range(num_dims):
                quant_origin[dim] = quantization_origin[dim]
        binary_metadata = encode_metadata(metadata)
        encoded_point_cloud = DracoPy.encode_point_cloud(
            points, binary_metadata, quantization_bits, compression_level,
            quantization_range, quant_origin, create_metadata)
        if quant_origin != NULL:
            PyMem_Free(quant_origin)
        if encoded_point_cloud.encode_status == DracoPy.encoding_status.successful_encoding:
            return bytes(encoded_point_cloud.buffer)
        elif encoded_point_cloud.encode_status == DracoPy.encoding_status.failed_during_encoding:
            raise EncodingFailedException('Invalid point cloud')
    except EncodingFailedException:
        raise EncodingFailedException('Invalid point cloud')
    except:
        if quant_origin != NULL:
            PyMem_Free(quant_origin)
        raise ValueError("Input invalid")

def raise_decoding_error(decoding_status):
    if decoding_status == DracoPy.decoding_status.not_draco_encoded:
        raise FileTypeException('Input mesh is not draco encoded')
    elif decoding_status == DracoPy.decoding_status.failed_during_decoding:
        raise TypeError('Failed to decode input mesh. Data might be corrupted')
    elif decoding_status == DracoPy.decoding_status.no_position_attribute:
        raise ValueError('DracoPy only supports meshes with position attributes')

def decode_buffer_to_mesh(buffer):
    mesh_struct = DracoPy.decode_buffer(buffer, len(buffer))
    if mesh_struct.decode_status == DracoPy.decoding_status.successful:
        return DracoMesh(mesh_struct)
    else:
        raise_decoding_error(mesh_struct.decode_status)

def decode_point_cloud_buffer(buffer):
    point_cloud_struct = DracoPy.decode_buffer_to_point_cloud(buffer, len(buffer))
    if point_cloud_struct.decode_status == DracoPy.decoding_status.successful:
        return DracoPointCloud(point_cloud_struct)
    else:
        raise_decoding_error(point_cloud_struct.decode_status)