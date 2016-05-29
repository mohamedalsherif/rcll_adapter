"""autogenerated by genpy from rcll_adapter/TagVision.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import rcll_adapter.msg
import geometry_msgs.msg
import std_msgs.msg

class TagVision(genpy.Message):
  _md5sum = "25f564daf8491a4bc7dff2c7b8b8258f"
  _type = "rcll_adapter/TagVision"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """std_msgs/Header header	# Reference coordinate frame for the data.
int32 tags_visible 		# The number of currently visible tags
int32[12] tag_id 		# The IDs of the tags

Position3D[] position # should we add those here ?

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: rcll_adapter/Position3D
int32 visibility_history
geometry_msgs/PoseWithCovarianceStamped pose_with_cov_stamped
================================================================================
MSG: geometry_msgs/PoseWithCovarianceStamped
# This expresses an estimated pose with a reference coordinate frame and timestamp

Header header
PoseWithCovariance pose

================================================================================
MSG: geometry_msgs/PoseWithCovariance
# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of postion and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

"""
  __slots__ = ['header','tags_visible','tag_id','position']
  _slot_types = ['std_msgs/Header','int32','int32[12]','rcll_adapter/Position3D[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,tags_visible,tag_id,position

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(TagVision, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.tags_visible is None:
        self.tags_visible = 0
      if self.tag_id is None:
        self.tag_id = [0,0,0,0,0,0,0,0,0,0,0,0]
      if self.position is None:
        self.position = []
    else:
      self.header = std_msgs.msg.Header()
      self.tags_visible = 0
      self.tag_id = [0,0,0,0,0,0,0,0,0,0,0,0]
      self.position = []

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_i.pack(self.tags_visible))
      buff.write(_struct_12i.pack(*self.tag_id))
      length = len(self.position)
      buff.write(_struct_I.pack(length))
      for val1 in self.position:
        buff.write(_struct_i.pack(val1.visibility_history))
        _v1 = val1.pose_with_cov_stamped
        _v2 = _v1.header
        buff.write(_struct_I.pack(_v2.seq))
        _v3 = _v2.stamp
        _x = _v3
        buff.write(_struct_2I.pack(_x.secs, _x.nsecs))
        _x = _v2.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _v4 = _v1.pose
        _v5 = _v4.pose
        _v6 = _v5.position
        _x = _v6
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v7 = _v5.orientation
        _x = _v7
        buff.write(_struct_4d.pack(_x.x, _x.y, _x.z, _x.w))
        buff.write(_struct_36d.pack(*_v4.covariance))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.position is None:
        self.position = None
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      start = end
      end += 4
      (self.tags_visible,) = _struct_i.unpack(str[start:end])
      start = end
      end += 48
      self.tag_id = _struct_12i.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.position = []
      for i in range(0, length):
        val1 = rcll_adapter.msg.Position3D()
        start = end
        end += 4
        (val1.visibility_history,) = _struct_i.unpack(str[start:end])
        _v8 = val1.pose_with_cov_stamped
        _v9 = _v8.header
        start = end
        end += 4
        (_v9.seq,) = _struct_I.unpack(str[start:end])
        _v10 = _v9.stamp
        _x = _v10
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _struct_2I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v9.frame_id = str[start:end].decode('utf-8')
        else:
          _v9.frame_id = str[start:end]
        _v11 = _v8.pose
        _v12 = _v11.pose
        _v13 = _v12.position
        _x = _v13
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _v14 = _v12.orientation
        _x = _v14
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _struct_4d.unpack(str[start:end])
        start = end
        end += 288
        _v11.covariance = _struct_36d.unpack(str[start:end])
        self.position.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_i.pack(self.tags_visible))
      buff.write(self.tag_id.tostring())
      length = len(self.position)
      buff.write(_struct_I.pack(length))
      for val1 in self.position:
        buff.write(_struct_i.pack(val1.visibility_history))
        _v15 = val1.pose_with_cov_stamped
        _v16 = _v15.header
        buff.write(_struct_I.pack(_v16.seq))
        _v17 = _v16.stamp
        _x = _v17
        buff.write(_struct_2I.pack(_x.secs, _x.nsecs))
        _x = _v16.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _v18 = _v15.pose
        _v19 = _v18.pose
        _v20 = _v19.position
        _x = _v20
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v21 = _v19.orientation
        _x = _v21
        buff.write(_struct_4d.pack(_x.x, _x.y, _x.z, _x.w))
        buff.write(_v18.covariance.tostring())
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.position is None:
        self.position = None
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      start = end
      end += 4
      (self.tags_visible,) = _struct_i.unpack(str[start:end])
      start = end
      end += 48
      self.tag_id = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=12)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.position = []
      for i in range(0, length):
        val1 = rcll_adapter.msg.Position3D()
        start = end
        end += 4
        (val1.visibility_history,) = _struct_i.unpack(str[start:end])
        _v22 = val1.pose_with_cov_stamped
        _v23 = _v22.header
        start = end
        end += 4
        (_v23.seq,) = _struct_I.unpack(str[start:end])
        _v24 = _v23.stamp
        _x = _v24
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _struct_2I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v23.frame_id = str[start:end].decode('utf-8')
        else:
          _v23.frame_id = str[start:end]
        _v25 = _v22.pose
        _v26 = _v25.pose
        _v27 = _v26.position
        _x = _v27
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _v28 = _v26.orientation
        _x = _v28
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _struct_4d.unpack(str[start:end])
        start = end
        end += 288
        _v25.covariance = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=36)
        self.position.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_i = struct.Struct("<i")
_struct_12i = struct.Struct("<12i")
_struct_36d = struct.Struct("<36d")
_struct_3I = struct.Struct("<3I")
_struct_4d = struct.Struct("<4d")
_struct_2I = struct.Struct("<2I")
_struct_3d = struct.Struct("<3d")