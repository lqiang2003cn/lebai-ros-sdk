# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from lebai_msgs/UntilInfo.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import lebai_msgs.msg

class UntilInfo(genpy.Message):
  _md5sum = "0907ad90f772aaa5fd1b8f10a92d018a"
  _type = "lebai_msgs/UntilInfo"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """uint8 io_express_logic
lebai_msgs/IOConditionalExpress[] io_express

uint8 LOGIC_AND=0
uint8 LOGIC_OR=1


================================================================================
MSG: lebai_msgs/IOConditionalExpress
uint32 group
uint32 pin
uint32 type
float64 float_value
uint8 uint_value
uint8 logic_operation

uint8 GROUP_ROBOT=0
uint8 GROUP_FLANGE=1

uint8 TYPE_ANALOG=0
uint8 TYPE_DIGITAL=1

# great >
uint8 LOGIC_OP_GT=0
# great and equal >=
uint8 LOGIC_OP_GE=1
# equal
uint8 LOGIC_OP_EQ=2
# not equal
uint8 LOGIC_OP_NE=3
# less than
uint8 LOGIC_OP_LT=4
# less than and equal
uint8 LOGIC_OP_LE=5



"""
  # Pseudo-constants
  LOGIC_AND = 0
  LOGIC_OR = 1

  __slots__ = ['io_express_logic','io_express']
  _slot_types = ['uint8','lebai_msgs/IOConditionalExpress[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       io_express_logic,io_express

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(UntilInfo, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.io_express_logic is None:
        self.io_express_logic = 0
      if self.io_express is None:
        self.io_express = []
    else:
      self.io_express_logic = 0
      self.io_express = []

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
      _x = self.io_express_logic
      buff.write(_get_struct_B().pack(_x))
      length = len(self.io_express)
      buff.write(_struct_I.pack(length))
      for val1 in self.io_express:
        _x = val1
        buff.write(_get_struct_3Id2B().pack(_x.group, _x.pin, _x.type, _x.float_value, _x.uint_value, _x.logic_operation))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.io_express is None:
        self.io_express = None
      end = 0
      start = end
      end += 1
      (self.io_express_logic,) = _get_struct_B().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.io_express = []
      for i in range(0, length):
        val1 = lebai_msgs.msg.IOConditionalExpress()
        _x = val1
        start = end
        end += 22
        (_x.group, _x.pin, _x.type, _x.float_value, _x.uint_value, _x.logic_operation,) = _get_struct_3Id2B().unpack(str[start:end])
        self.io_express.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self.io_express_logic
      buff.write(_get_struct_B().pack(_x))
      length = len(self.io_express)
      buff.write(_struct_I.pack(length))
      for val1 in self.io_express:
        _x = val1
        buff.write(_get_struct_3Id2B().pack(_x.group, _x.pin, _x.type, _x.float_value, _x.uint_value, _x.logic_operation))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.io_express is None:
        self.io_express = None
      end = 0
      start = end
      end += 1
      (self.io_express_logic,) = _get_struct_B().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.io_express = []
      for i in range(0, length):
        val1 = lebai_msgs.msg.IOConditionalExpress()
        _x = val1
        start = end
        end += 22
        (_x.group, _x.pin, _x.type, _x.float_value, _x.uint_value, _x.logic_operation,) = _get_struct_3Id2B().unpack(str[start:end])
        self.io_express.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3Id2B = None
def _get_struct_3Id2B():
    global _struct_3Id2B
    if _struct_3Id2B is None:
        _struct_3Id2B = struct.Struct("<3Id2B")
    return _struct_3Id2B
_struct_B = None
def _get_struct_B():
    global _struct_B
    if _struct_B is None:
        _struct_B = struct.Struct("<B")
    return _struct_B
