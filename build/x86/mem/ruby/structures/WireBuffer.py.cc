#include "sim/init.hh"

namespace {

const uint8_t data_m5_objects_WireBuffer[] = {
    120,156,189,144,79,75,195,64,16,197,103,211,84,180,136,120,
    22,132,122,11,66,186,254,235,65,16,145,126,0,133,68,17,
    123,9,219,205,164,137,100,211,178,187,129,246,172,223,91,103,
    82,107,143,222,220,36,143,55,48,59,243,203,211,240,115,122,
    244,61,12,1,220,13,153,156,94,1,53,192,51,187,0,106,
    1,38,128,105,0,34,239,1,6,80,8,200,67,248,4,248,
    0,120,155,246,32,239,67,26,237,209,197,234,139,78,36,200,
    121,150,243,141,61,32,73,43,243,52,123,71,237,253,17,85,
    73,59,91,191,86,22,39,109,81,160,213,91,8,238,158,48,
    196,9,25,4,152,10,70,161,189,68,195,107,136,33,132,36,
    141,24,54,9,120,244,128,100,55,200,157,81,105,208,72,75,
    243,165,243,182,213,190,181,232,228,174,101,84,150,81,159,111,
    238,147,100,89,163,12,102,89,55,38,203,204,34,111,107,46,
    67,110,88,47,177,35,215,171,85,166,107,229,92,215,197,85,
    137,42,71,27,49,241,78,220,11,137,52,141,151,229,188,112,
    50,45,149,197,180,196,70,206,209,140,227,133,173,230,85,19,
    59,175,102,53,198,87,23,151,227,248,54,190,150,206,106,249,
    7,239,114,221,253,233,144,87,112,196,123,130,159,199,40,220,
    38,107,198,163,165,178,202,56,127,184,169,126,163,78,196,54,
    167,127,99,237,98,189,219,4,121,127,202,171,152,115,32,142,
    131,111,20,87,145,138,
};

EmbeddedPython embedded_m5_objects_WireBuffer(
    "m5/objects/WireBuffer.py",
    "/mnt/hgfs/ShareShen/gem5-origin-stable-2015-9-3/src/mem/ruby/structures/WireBuffer.py",
    "m5.objects.WireBuffer",
    data_m5_objects_WireBuffer,
    326,
    618);

} // anonymous namespace
