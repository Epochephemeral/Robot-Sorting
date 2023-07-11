# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from dobot/SetCPParamsRequest.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class SetCPParamsRequest(genpy.Message):
  _md5sum = "554ea4f3eb746a3d6db4a5ca9e210a01"
  _type = "dobot/SetCPParamsRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """float32 planAcc
float32 junctionVel
float32 acc
uint8 realTimeTrack
bool isQueued
"""
  __slots__ = ['planAcc','junctionVel','acc','realTimeTrack','isQueued']
  _slot_types = ['float32','float32','float32','uint8','bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       planAcc,junctionVel,acc,realTimeTrack,isQueued

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(SetCPParamsRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.planAcc is None:
        self.planAcc = 0.
      if self.junctionVel is None:
        self.junctionVel = 0.
      if self.acc is None:
        self.acc = 0.
      if self.realTimeTrack is None:
        self.realTimeTrack = 0
      if self.isQueued is None:
        self.isQueued = False
    else:
      self.planAcc = 0.
      self.junctionVel = 0.
      self.acc = 0.
      self.realTimeTrack = 0
      self.isQueued = False

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
      buff.write(_get_struct_3f2B().pack(_x.planAcc, _x.junctionVel, _x.acc, _x.realTimeTrack, _x.isQueued))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 14
      (_x.planAcc, _x.junctionVel, _x.acc, _x.realTimeTrack, _x.isQueued,) = _get_struct_3f2B().unpack(str[start:end])
      self.isQueued = bool(self.isQueued)
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
      buff.write(_get_struct_3f2B().pack(_x.planAcc, _x.junctionVel, _x.acc, _x.realTimeTrack, _x.isQueued))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

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
      end += 14
      (_x.planAcc, _x.junctionVel, _x.acc, _x.realTimeTrack, _x.isQueued,) = _get_struct_3f2B().unpack(str[start:end])
      self.isQueued = bool(self.isQueued)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3f2B = None
def _get_struct_3f2B():
    global _struct_3f2B
    if _struct_3f2B is None:
        _struct_3f2B = struct.Struct("<3f2B")
    return _struct_3f2B
# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from dobot/SetCPParamsResponse.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class SetCPParamsResponse(genpy.Message):
  _md5sum = "cbf7b461449133eb5dd6ebbd8d38dedc"
  _type = "dobot/SetCPParamsResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """int32 result
uint64 queuedCmdIndex

"""
  __slots__ = ['result','queuedCmdIndex']
  _slot_types = ['int32','uint64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       result,queuedCmdIndex

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(SetCPParamsResponse, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.result is None:
        self.result = 0
      if self.queuedCmdIndex is None:
        self.queuedCmdIndex = 0
    else:
      self.result = 0
      self.queuedCmdIndex = 0

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
      buff.write(_get_struct_iQ().pack(_x.result, _x.queuedCmdIndex))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 12
      (_x.result, _x.queuedCmdIndex,) = _get_struct_iQ().unpack(str[start:end])
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
      buff.write(_get_struct_iQ().pack(_x.result, _x.queuedCmdIndex))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

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
      end += 12
      (_x.result, _x.queuedCmdIndex,) = _get_struct_iQ().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_iQ = None
def _get_struct_iQ():
    global _struct_iQ
    if _struct_iQ is None:
        _struct_iQ = struct.Struct("<iQ")
    return _struct_iQ
class SetCPParams(object):
  _type          = 'dobot/SetCPParams'
  _md5sum = 'f1c22d901d2cad5a8687473723199ac1'
  _request_class  = SetCPParamsRequest
  _response_class = SetCPParamsResponse