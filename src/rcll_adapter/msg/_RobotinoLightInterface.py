"""autogenerated by genpy from rcll_adapter/RobotinoLightInterface.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class RobotinoLightInterface(genpy.Message):
  _md5sum = "0b87542e74281d461cf98cb52edfb043"
  _type = "rcll_adapter/RobotinoLightInterface"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """uint8 LIGHT_STATE_ON = 0 		# the signal is on
uint8 LIGHT_STATE_OFF = 1 		# the signal is off
uint8 LIGHT_STATE_BLINKING = 2  # the signal is blinking.
uint8 LIGHT_STATE_UKNOWN = 3 	# the signal state is unknown
uint8 red 						# State of red light
uint8 yellow 					# State of yellow light
uint8 green 					# State of green light

int32 visibility_history 		# visibility history 
bool ready 						# Data valid and ready
"""
  # Pseudo-constants
  LIGHT_STATE_ON = 0
  LIGHT_STATE_OFF = 1
  LIGHT_STATE_BLINKING = 2
  LIGHT_STATE_UKNOWN = 3

  __slots__ = ['red','yellow','green','visibility_history','ready']
  _slot_types = ['uint8','uint8','uint8','int32','bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       red,yellow,green,visibility_history,ready

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(RobotinoLightInterface, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.red is None:
        self.red = 0
      if self.yellow is None:
        self.yellow = 0
      if self.green is None:
        self.green = 0
      if self.visibility_history is None:
        self.visibility_history = 0
      if self.ready is None:
        self.ready = False
    else:
      self.red = 0
      self.yellow = 0
      self.green = 0
      self.visibility_history = 0
      self.ready = False

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
      buff.write(_struct_3BiB.pack(_x.red, _x.yellow, _x.green, _x.visibility_history, _x.ready))
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
      end += 8
      (_x.red, _x.yellow, _x.green, _x.visibility_history, _x.ready,) = _struct_3BiB.unpack(str[start:end])
      self.ready = bool(self.ready)
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
      buff.write(_struct_3BiB.pack(_x.red, _x.yellow, _x.green, _x.visibility_history, _x.ready))
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
      end += 8
      (_x.red, _x.yellow, _x.green, _x.visibility_history, _x.ready,) = _struct_3BiB.unpack(str[start:end])
      self.ready = bool(self.ready)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_3BiB = struct.Struct("<3BiB")
