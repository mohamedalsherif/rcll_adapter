"""autogenerated by genpy from rcll_adapter/Position3DInterface.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class Position3DInterface(genpy.Message):
  _md5sum = "7a04833f99b7e161a898c3c358ad6702"
  _type = "rcll_adapter/Position3DInterface"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """string[32] frame				
#  Reference coordinate frame for the data.

int32 visibility_history	
#  The visibilitiy history indicates the number of consecutive positive or negative
#  sightings. If the history is negative, there have been as many negative sightings
#  (object not visible) as the absolute value of the history. A positive value denotes
#  as many positive sightings. 0 shall only be used during the initialization of the
#  interface or if the visibility history is not updated.

float64[4] rotation
#  Rotation quaternion relative to reference frame, ordered as (x, y, z, w).

float64[3] translation
#  Translation vector from the reference frame's origin, ordered as (x, y, z).

float64[36] covariance
#  Row-major representation of the 6x6 covariance matrix.
#  The orientation parameters use a fixed-axis representation.
#  In order, the parameters are:
#  (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
"""
  __slots__ = ['frame','visibility_history','rotation','translation','covariance']
  _slot_types = ['string[32]','int32','float64[4]','float64[3]','float64[36]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       frame,visibility_history,rotation,translation,covariance

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Position3DInterface, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.frame is None:
        self.frame = ['','','','','','','','','','','','','','','','','','','','','','','','','','','','','','','','']
      if self.visibility_history is None:
        self.visibility_history = 0
      if self.rotation is None:
        self.rotation = [0.,0.,0.,0.]
      if self.translation is None:
        self.translation = [0.,0.,0.]
      if self.covariance is None:
        self.covariance = [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
    else:
      self.frame = ['','','','','','','','','','','','','','','','','','','','','','','','','','','','','','','','']
      self.visibility_history = 0
      self.rotation = [0.,0.,0.,0.]
      self.translation = [0.,0.,0.]
      self.covariance = [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]

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
      for val1 in self.frame:
        length = len(val1)
        if python3 or type(val1) == unicode:
          val1 = val1.encode('utf-8')
          length = len(val1)
        buff.write(struct.pack('<I%ss'%length, length, val1))
      buff.write(_struct_i.pack(self.visibility_history))
      buff.write(_struct_4d.pack(*self.rotation))
      buff.write(_struct_3d.pack(*self.translation))
      buff.write(_struct_36d.pack(*self.covariance))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      self.frame = []
      for i in range(0, 32):
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1 = str[start:end].decode('utf-8')
        else:
          val1 = str[start:end]
        self.frame.append(val1)
      start = end
      end += 4
      (self.visibility_history,) = _struct_i.unpack(str[start:end])
      start = end
      end += 32
      self.rotation = _struct_4d.unpack(str[start:end])
      start = end
      end += 24
      self.translation = _struct_3d.unpack(str[start:end])
      start = end
      end += 288
      self.covariance = _struct_36d.unpack(str[start:end])
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
      for val1 in self.frame:
        length = len(val1)
        if python3 or type(val1) == unicode:
          val1 = val1.encode('utf-8')
          length = len(val1)
        buff.write(struct.pack('<I%ss'%length, length, val1))
      buff.write(_struct_i.pack(self.visibility_history))
      buff.write(self.rotation.tostring())
      buff.write(self.translation.tostring())
      buff.write(self.covariance.tostring())
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      self.frame = []
      for i in range(0, 32):
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1 = str[start:end].decode('utf-8')
        else:
          val1 = str[start:end]
        self.frame.append(val1)
      start = end
      end += 4
      (self.visibility_history,) = _struct_i.unpack(str[start:end])
      start = end
      end += 32
      self.rotation = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=4)
      start = end
      end += 24
      self.translation = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=3)
      start = end
      end += 288
      self.covariance = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=36)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_i = struct.Struct("<i")
_struct_4d = struct.Struct("<4d")
_struct_36d = struct.Struct("<36d")
_struct_3d = struct.Struct("<3d")