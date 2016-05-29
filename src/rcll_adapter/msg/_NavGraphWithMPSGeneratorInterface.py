"""autogenerated by genpy from rcll_adapter/NavGraphWithMPSGeneratorInterface.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class NavGraphWithMPSGeneratorInterface(genpy.Message):
  _md5sum = "3ed417c3681a1e68958d568c22841adc"
  _type = "rcll_adapter/NavGraphWithMPSGeneratorInterface"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """uint32 	msgid 
#  The ID of the message that is currently being processed or
#  was processed last.

bool final 
#  True, if the last generation triggered by a ComputeMessage has
#  been completed, false if it is still running. Also check the
#  msgid field if this field applies to the correct message.
"""
  __slots__ = ['msgid','final']
  _slot_types = ['uint32','bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       msgid,final

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(NavGraphWithMPSGeneratorInterface, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.msgid is None:
        self.msgid = 0
      if self.final is None:
        self.final = False
    else:
      self.msgid = 0
      self.final = False

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
      buff.write(_struct_IB.pack(_x.msgid, _x.final))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 5
      (_x.msgid, _x.final,) = _struct_IB.unpack(str[start:end])
      self.final = bool(self.final)
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
      buff.write(_struct_IB.pack(_x.msgid, _x.final))
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
      _x = self
      start = end
      end += 5
      (_x.msgid, _x.final,) = _struct_IB.unpack(str[start:end])
      self.final = bool(self.final)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_IB = struct.Struct("<IB")
