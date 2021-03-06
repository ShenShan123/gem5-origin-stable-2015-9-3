# This file was automatically generated by SWIG (http://www.swig.org).
# Version 3.0.8
#
# Do not make changes to this file unless you know what you are doing--modify
# the SWIG interface file instead.





from sys import version_info
if version_info >= (2, 6, 0):
    def swig_import_helper():
        from os.path import dirname
        import imp
        fp = None
        try:
            fp, pathname, description = imp.find_module('_param_RubyMemoryControl', [dirname(__file__)])
        except ImportError:
            import _param_RubyMemoryControl
            return _param_RubyMemoryControl
        if fp is not None:
            try:
                _mod = imp.load_module('_param_RubyMemoryControl', fp, pathname, description)
            finally:
                fp.close()
            return _mod
    _param_RubyMemoryControl = swig_import_helper()
    del swig_import_helper
else:
    import _param_RubyMemoryControl
del version_info
try:
    _swig_property = property
except NameError:
    pass  # Python < 2.2 doesn't have 'property'.


def _swig_setattr_nondynamic(self, class_type, name, value, static=1):
    if (name == "thisown"):
        return self.this.own(value)
    if (name == "this"):
        if type(value).__name__ == 'SwigPyObject':
            self.__dict__[name] = value
            return
    method = class_type.__swig_setmethods__.get(name, None)
    if method:
        return method(self, value)
    if (not static):
        object.__setattr__(self, name, value)
    else:
        raise AttributeError("You cannot add attributes to %s" % self)


def _swig_setattr(self, class_type, name, value):
    return _swig_setattr_nondynamic(self, class_type, name, value, 0)


def _swig_getattr_nondynamic(self, class_type, name, static=1):
    if (name == "thisown"):
        return self.this.own()
    method = class_type.__swig_getmethods__.get(name, None)
    if method:
        return method(self)
    if (not static):
        return object.__getattr__(self, name)
    else:
        raise AttributeError(name)

def _swig_getattr(self, class_type, name):
    return _swig_getattr_nondynamic(self, class_type, name, 0)


def _swig_repr(self):
    try:
        strthis = "proxy of " + self.this.__repr__()
    except Exception:
        strthis = ""
    return "<%s.%s; %s >" % (self.__class__.__module__, self.__class__.__name__, strthis,)

try:
    _object = object
    _newclass = 1
except AttributeError:
    class _object:
        pass
    _newclass = 0



def _swig_setattr_nondynamic_method(set):
    def set_attr(self, name, value):
        if (name == "thisown"):
            return self.this.own(value)
        if hasattr(self, name) or (name == "this"):
            set(self, name, value)
        else:
            raise AttributeError("You cannot add attributes to %s" % self)
    return set_attr


import m5.internal.param_AbstractMemory
import m5.internal.param_MemObject
import m5.internal.param_ClockedObject
import m5.internal.param_ClockDomain
import m5.internal.param_SimObject
import m5.internal.drain
import m5.internal.serialize
class RubyMemoryControl(m5.internal.param_AbstractMemory.AbstractMemory):
    thisown = _swig_property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')

    def __init__(self, *args, **kwargs):
        raise AttributeError("No constructor defined - class is abstract")
    __repr__ = _swig_repr
RubyMemoryControl_swigregister = _param_RubyMemoryControl.RubyMemoryControl_swigregister
RubyMemoryControl_swigregister(RubyMemoryControl)

class RubyMemoryControlParams(m5.internal.param_AbstractMemory.AbstractMemoryParams):
    thisown = _swig_property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')
    __repr__ = _swig_repr

    def create(self):
        return _param_RubyMemoryControl.RubyMemoryControlParams_create(self)
    bank_bit_0 = _swig_property(_param_RubyMemoryControl.RubyMemoryControlParams_bank_bit_0_get, _param_RubyMemoryControl.RubyMemoryControlParams_bank_bit_0_set)
    bank_busy_time = _swig_property(_param_RubyMemoryControl.RubyMemoryControlParams_bank_busy_time_get, _param_RubyMemoryControl.RubyMemoryControlParams_bank_busy_time_set)
    bank_queue_size = _swig_property(_param_RubyMemoryControl.RubyMemoryControlParams_bank_queue_size_get, _param_RubyMemoryControl.RubyMemoryControlParams_bank_queue_size_set)
    banks_per_rank = _swig_property(_param_RubyMemoryControl.RubyMemoryControlParams_banks_per_rank_get, _param_RubyMemoryControl.RubyMemoryControlParams_banks_per_rank_set)
    basic_bus_busy_time = _swig_property(_param_RubyMemoryControl.RubyMemoryControlParams_basic_bus_busy_time_get, _param_RubyMemoryControl.RubyMemoryControlParams_basic_bus_busy_time_set)
    dimm_bit_0 = _swig_property(_param_RubyMemoryControl.RubyMemoryControlParams_dimm_bit_0_get, _param_RubyMemoryControl.RubyMemoryControlParams_dimm_bit_0_set)
    dimms_per_channel = _swig_property(_param_RubyMemoryControl.RubyMemoryControlParams_dimms_per_channel_get, _param_RubyMemoryControl.RubyMemoryControlParams_dimms_per_channel_set)
    mem_ctl_latency = _swig_property(_param_RubyMemoryControl.RubyMemoryControlParams_mem_ctl_latency_get, _param_RubyMemoryControl.RubyMemoryControlParams_mem_ctl_latency_set)
    mem_fixed_delay = _swig_property(_param_RubyMemoryControl.RubyMemoryControlParams_mem_fixed_delay_get, _param_RubyMemoryControl.RubyMemoryControlParams_mem_fixed_delay_set)
    mem_random_arbitrate = _swig_property(_param_RubyMemoryControl.RubyMemoryControlParams_mem_random_arbitrate_get, _param_RubyMemoryControl.RubyMemoryControlParams_mem_random_arbitrate_set)
    rank_bit_0 = _swig_property(_param_RubyMemoryControl.RubyMemoryControlParams_rank_bit_0_get, _param_RubyMemoryControl.RubyMemoryControlParams_rank_bit_0_set)
    rank_rank_delay = _swig_property(_param_RubyMemoryControl.RubyMemoryControlParams_rank_rank_delay_get, _param_RubyMemoryControl.RubyMemoryControlParams_rank_rank_delay_set)
    ranks_per_dimm = _swig_property(_param_RubyMemoryControl.RubyMemoryControlParams_ranks_per_dimm_get, _param_RubyMemoryControl.RubyMemoryControlParams_ranks_per_dimm_set)
    read_write_delay = _swig_property(_param_RubyMemoryControl.RubyMemoryControlParams_read_write_delay_get, _param_RubyMemoryControl.RubyMemoryControlParams_read_write_delay_set)
    refresh_period = _swig_property(_param_RubyMemoryControl.RubyMemoryControlParams_refresh_period_get, _param_RubyMemoryControl.RubyMemoryControlParams_refresh_period_set)
    tFaw = _swig_property(_param_RubyMemoryControl.RubyMemoryControlParams_tFaw_get, _param_RubyMemoryControl.RubyMemoryControlParams_tFaw_set)
    port_port_connection_count = _swig_property(_param_RubyMemoryControl.RubyMemoryControlParams_port_port_connection_count_get, _param_RubyMemoryControl.RubyMemoryControlParams_port_port_connection_count_set)

    def __init__(self):
        this = _param_RubyMemoryControl.new_RubyMemoryControlParams()
        try:
            self.this.append(this)
        except Exception:
            self.this = this
    __swig_destroy__ = _param_RubyMemoryControl.delete_RubyMemoryControlParams
    __del__ = lambda self: None
RubyMemoryControlParams_swigregister = _param_RubyMemoryControl.RubyMemoryControlParams_swigregister
RubyMemoryControlParams_swigregister(RubyMemoryControlParams)



