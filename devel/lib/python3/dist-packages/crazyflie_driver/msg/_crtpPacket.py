# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from crazyflie_driver/crtpPacket.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class crtpPacket(genpy.Message):
  _md5sum = "211163da2417112110f499fc3a0bedf0"
  _type = "crazyflie_driver/crtpPacket"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """uint8 size
uint8 header
uint8[30] data
"""
  __slots__ = ['size','header','data']
  _slot_types = ['uint8','uint8','uint8[30]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       size,header,data

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(crtpPacket, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.size is None:
        self.size = 0
      if self.header is None:
        self.header = 0
      if self.data is None:
        self.data = b'\0'*30
    else:
      self.size = 0
      self.header = 0
      self.data = b'\0'*30

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
      buff.write(_get_struct_2B().pack(_x.size, _x.header))
      _x = self.data
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(_get_struct_30B().pack(*_x))
      else:
        buff.write(_get_struct_30s().pack(_x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 2
      (_x.size, _x.header,) = _get_struct_2B().unpack(str[start:end])
      start = end
      end += 30
      self.data = str[start:end]
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
      _x = self
      buff.write(_get_struct_2B().pack(_x.size, _x.header))
      _x = self.data
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(_get_struct_30B().pack(*_x))
      else:
        buff.write(_get_struct_30s().pack(_x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 2
      (_x.size, _x.header,) = _get_struct_2B().unpack(str[start:end])
      start = end
      end += 30
      self.data = str[start:end]
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2B = None
def _get_struct_2B():
    global _struct_2B
    if _struct_2B is None:
        _struct_2B = struct.Struct("<2B")
    return _struct_2B
_struct_30B = None
def _get_struct_30B():
    global _struct_30B
    if _struct_30B is None:
        _struct_30B = struct.Struct("<30B")
    return _struct_30B
_struct_30s = None
def _get_struct_30s():
    global _struct_30s
    if _struct_30s is None:
        _struct_30s = struct.Struct("<30s")
    return _struct_30s
