#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_RubyMemoryControl[] = {
    120,156,205,90,91,87,27,201,17,174,158,145,4,18,96,192,
    220,124,193,70,123,177,173,245,26,228,27,182,55,102,189,113,
    156,221,115,178,231,132,221,140,54,199,94,118,79,38,131,166,
    37,13,72,51,202,76,99,44,31,120,9,123,146,60,229,109,
    247,31,228,33,175,249,37,249,71,73,85,141,122,52,8,16,
    144,228,88,1,84,110,213,84,119,117,85,125,85,125,25,87,
    161,251,147,197,207,207,139,0,209,79,2,192,197,63,1,77,
    128,150,128,13,1,66,10,112,103,97,59,11,225,67,112,179,
    240,3,192,134,1,210,128,3,108,152,240,157,1,254,56,247,
    201,65,211,100,142,128,78,1,100,6,54,178,240,210,159,134,
    140,204,193,118,1,194,223,131,16,194,23,240,202,29,1,119,
    20,126,192,209,177,145,231,1,71,129,152,5,102,230,193,29,
    99,102,1,220,113,110,140,65,103,10,228,56,108,76,144,216,
    198,5,28,246,54,14,59,201,195,254,147,134,117,241,201,28,
    184,23,72,28,231,245,45,73,102,72,146,245,77,242,40,83,
    122,150,211,176,113,81,183,103,82,237,217,84,123,46,213,158,
    79,181,23,82,237,75,220,198,153,93,132,173,203,176,117,5,
    182,174,66,13,157,53,157,204,98,17,164,9,91,215,96,227,
    26,72,252,91,132,3,244,167,123,49,213,227,58,247,152,73,
    122,44,113,143,34,108,20,65,226,223,82,220,35,7,149,210,
    60,198,200,251,23,254,148,48,70,160,198,145,188,150,97,228,
    5,190,237,249,181,192,51,232,121,142,8,69,180,74,100,164,
    27,218,23,20,218,191,3,199,213,53,186,161,221,7,28,88,
    144,45,77,3,246,185,177,111,64,167,4,123,2,182,50,224,
    154,176,135,106,178,52,129,186,128,3,3,190,55,73,96,31,
    105,6,3,112,29,50,42,142,235,22,7,32,30,105,4,246,
    179,176,151,133,202,171,61,131,24,219,121,8,255,6,111,23,
    121,208,81,30,212,128,61,164,25,56,200,192,126,14,94,162,
    16,178,182,242,100,190,120,181,135,150,34,167,82,202,224,108,
    215,83,230,146,41,174,23,250,78,75,170,75,216,182,219,78,
    232,180,108,107,103,179,243,107,217,10,194,206,139,192,87,97,
    208,44,21,180,116,16,173,180,29,213,176,184,187,73,126,105,
    181,21,15,27,248,82,141,97,163,230,249,174,221,10,220,157,
    166,84,163,52,166,93,243,154,210,182,249,225,175,90,237,32,
    84,159,135,97,16,90,228,90,102,54,3,39,233,65,142,173,
    54,131,72,150,72,27,171,177,104,120,69,210,181,54,143,72,
    19,224,41,83,103,87,70,213,208,107,43,140,88,60,34,73,
    211,104,37,138,21,147,168,142,164,220,242,85,185,81,175,69,
    229,74,195,9,101,165,33,253,114,93,182,86,151,131,208,171,
    123,254,114,164,156,205,166,92,190,127,247,222,234,242,39,203,
    15,202,155,59,94,211,45,191,121,242,168,220,238,168,70,224,
    151,91,171,101,207,87,18,157,213,44,159,224,166,21,20,189,
    72,10,119,189,186,237,177,169,118,67,54,219,50,156,32,238,
    21,154,140,152,18,227,34,39,76,81,18,19,216,202,226,199,
    20,139,198,152,88,247,200,216,42,57,128,240,150,73,35,140,
    194,46,96,219,128,112,145,240,179,133,127,130,2,142,40,170,
    208,51,131,159,253,134,188,20,115,183,76,66,69,204,220,99,
    204,33,248,80,114,141,96,224,3,3,39,11,91,57,136,1,
    133,56,140,17,22,118,136,162,56,13,99,224,224,25,136,126,
    4,244,58,66,105,15,186,48,59,48,65,248,83,160,10,84,
    13,144,59,143,10,255,200,72,173,148,104,250,235,140,20,213,
    240,162,96,215,231,120,80,155,115,171,130,158,249,186,243,213,
    230,150,172,170,104,9,25,223,6,59,197,170,227,251,129,42,
    58,174,91,116,148,10,189,205,29,37,163,162,10,138,55,162,
    18,133,216,154,214,96,75,198,235,180,53,184,8,8,8,174,
    248,139,235,85,21,126,153,225,47,28,133,72,42,4,74,35,
    112,35,228,211,16,117,169,44,154,164,34,39,7,60,17,198,
    145,77,162,164,30,229,46,224,247,231,122,38,12,214,82,78,
    67,43,146,205,154,42,48,74,157,40,178,121,38,196,103,64,
    210,192,175,157,230,142,228,209,17,82,10,39,68,205,120,14,
    67,130,36,103,182,246,6,155,232,7,190,219,193,25,123,213,
    91,52,153,75,12,204,113,134,230,28,194,114,4,105,14,255,
    205,137,121,163,154,233,130,49,167,1,73,37,83,1,195,65,
    116,17,129,224,60,192,242,84,50,184,190,176,149,156,185,239,
    83,139,58,91,139,68,174,17,185,78,100,73,59,226,221,123,
    99,162,223,27,143,105,6,6,187,128,141,165,32,154,218,88,
    247,80,246,93,238,101,31,22,214,10,101,145,65,185,214,203,
    162,12,21,225,240,25,81,20,229,252,52,33,250,134,74,62,
    101,27,15,70,137,133,41,66,173,94,226,176,235,172,41,114,
    201,168,198,188,69,64,78,163,185,158,66,179,69,81,99,40,
    91,151,117,37,181,73,34,6,177,117,149,134,202,30,227,251,
    34,145,247,134,23,128,30,28,235,71,224,248,148,38,51,213,
    133,227,4,195,176,128,159,41,163,106,118,163,146,172,188,51,
    125,48,36,12,102,142,193,224,77,106,153,71,253,48,116,248,
    117,173,255,34,5,63,154,176,145,54,114,29,27,157,5,178,
    45,13,188,5,220,88,188,244,23,112,175,96,240,94,225,46,
    239,21,120,191,193,59,179,184,224,155,92,243,227,70,150,156,
    84,51,97,190,187,7,136,242,72,219,97,240,166,83,12,106,
    69,197,94,160,250,188,118,35,90,185,17,61,197,202,91,124,
    198,53,47,174,189,113,117,13,101,155,170,35,117,253,252,77,
    85,242,154,203,223,108,59,46,134,54,23,70,187,187,150,35,
    6,231,200,197,134,246,61,47,11,145,10,105,53,24,146,247,
    11,137,247,201,152,47,73,125,129,93,111,138,5,196,91,65,
    240,28,237,120,93,224,221,30,63,197,207,47,40,28,228,7,
    9,180,127,183,42,177,5,108,28,153,105,221,57,132,169,119,
    110,154,85,70,93,191,213,88,202,245,176,68,31,83,39,204,
    159,129,119,198,2,254,4,132,22,4,69,55,97,146,252,34,
    120,204,144,248,239,128,51,235,152,125,7,215,173,10,237,53,
    88,2,203,89,244,152,69,227,109,200,151,240,151,84,90,234,
    205,130,217,221,251,166,55,11,153,164,230,49,204,206,180,33,
    200,28,46,142,20,174,134,19,145,88,92,241,122,153,222,91,
    104,146,205,42,86,252,119,143,185,209,88,177,77,115,252,190,
    135,56,90,110,175,138,25,35,133,163,123,68,238,39,16,18,
    154,247,78,167,187,4,39,239,22,236,120,245,249,142,230,148,
    97,43,38,71,20,7,163,127,164,36,119,178,58,119,238,39,
    185,35,121,181,252,129,15,78,68,13,130,198,129,33,240,148,
    139,27,74,58,84,102,64,102,97,35,71,89,198,7,2,209,
    77,66,161,235,34,85,209,67,75,49,59,107,61,118,99,130,
    142,56,240,68,222,12,169,222,80,236,215,154,78,107,211,117,
    158,249,164,156,102,80,213,105,105,104,115,166,210,230,80,74,
    137,147,44,226,175,171,218,172,215,67,170,53,143,80,87,98,
    14,103,150,27,84,185,192,124,211,144,197,150,108,109,226,25,
    186,225,181,139,181,166,83,231,232,153,93,115,191,210,230,42,
    14,127,255,6,40,186,77,52,40,86,3,31,23,138,157,170,
    10,194,162,43,241,72,41,221,226,114,145,87,153,162,23,21,
    157,77,124,234,84,85,156,34,135,243,158,119,227,78,88,143,
    120,227,189,189,75,205,33,70,223,182,61,223,195,67,73,27,
    146,101,62,62,218,38,139,6,31,55,226,140,195,21,25,15,
    139,170,19,215,68,218,12,89,43,68,62,130,225,174,45,15,
    81,87,139,148,146,75,115,226,170,145,55,212,194,113,105,255,
    53,141,17,29,77,254,127,136,51,36,127,124,185,213,45,1,
    57,146,148,35,116,191,65,52,79,203,204,70,65,51,199,152,
    142,51,115,66,51,47,48,157,100,230,148,102,78,51,189,200,
    204,25,205,156,101,58,199,204,121,205,92,96,122,137,153,151,
    53,243,10,211,171,204,92,212,204,107,76,175,51,115,73,51,
    139,76,223,99,230,251,154,249,1,211,15,153,121,67,51,111,
    50,189,197,204,146,102,126,196,244,54,51,63,214,204,59,76,
    151,153,185,162,153,101,166,119,153,121,79,51,239,51,125,192,
    204,135,154,185,202,244,17,51,31,107,230,19,166,159,48,243,
    103,250,234,241,41,51,215,96,227,83,186,131,35,206,51,42,
    190,35,255,109,241,229,82,53,196,34,181,251,63,173,185,214,
    227,255,19,107,172,39,208,221,208,157,84,111,69,218,212,137,
    184,222,110,9,125,38,77,219,201,151,102,215,79,78,102,187,
    26,74,71,201,56,156,139,67,115,0,87,243,120,42,111,123,
    165,244,232,137,233,121,98,235,1,111,113,59,179,28,229,248,
    152,206,81,22,47,253,43,120,116,202,240,209,105,141,142,78,
    123,236,24,219,136,79,79,61,64,103,19,255,208,18,227,203,
    221,163,51,139,125,20,159,146,104,134,78,187,45,125,215,186,
    13,233,131,15,63,30,18,110,104,17,249,43,164,118,157,166,
    152,197,147,206,209,172,166,101,51,101,59,135,59,155,228,241,
    240,2,207,200,255,81,35,191,244,25,164,215,78,107,141,8,
    175,150,201,66,105,125,150,132,237,230,0,88,111,58,254,182,
    189,137,75,243,93,58,137,159,85,20,247,196,124,136,236,177,
    212,237,83,123,238,68,29,91,121,45,201,138,206,33,78,202,
    46,36,202,52,91,125,124,218,8,127,216,145,59,216,219,123,
    27,107,60,143,60,169,156,212,42,123,252,83,103,29,217,184,
    113,177,67,234,117,22,35,83,226,105,35,123,108,85,30,56,
    66,132,103,17,116,72,159,107,207,219,135,84,207,176,234,35,
    207,6,226,193,245,90,173,51,66,39,37,170,161,211,99,169,
    229,83,122,198,254,168,54,240,60,44,155,172,235,124,61,72,
    229,116,87,229,161,39,3,49,129,235,137,93,85,77,187,137,
    181,214,175,118,78,197,80,191,188,198,80,31,255,212,49,106,
    222,27,233,218,174,108,58,103,211,153,150,79,235,76,241,213,
    221,83,198,64,172,185,65,203,118,66,12,72,136,243,100,197,
    231,238,68,218,103,187,218,251,31,14,132,71,120,246,34,20,
    30,45,66,61,214,64,95,177,24,147,179,249,182,95,94,251,
    182,143,63,48,201,195,36,155,25,238,167,213,132,62,113,93,
    19,14,179,213,157,65,35,72,199,181,119,67,15,163,209,179,
    242,92,29,72,41,173,36,253,15,6,79,92,214,66,25,53,
    104,142,94,224,158,110,231,97,241,196,206,67,108,85,28,48,
    130,250,194,217,101,53,167,10,209,224,252,178,12,191,168,71,
    3,164,249,61,37,19,60,118,251,178,74,247,186,216,220,241,
    21,43,250,15,187,146,122,122,227,121,178,8,239,77,120,235,
    137,126,150,232,238,19,180,112,84,186,111,64,92,137,39,255,
    160,99,219,241,69,45,69,200,182,135,121,52,254,20,117,189,
    38,165,180,255,192,163,177,200,225,225,120,78,156,242,107,228,
    115,121,193,119,22,125,255,195,32,182,130,222,206,196,23,150,
    157,200,34,142,53,153,236,105,248,13,184,190,39,160,237,15,
    95,193,175,59,173,248,77,37,191,115,179,62,128,238,155,15,
    235,86,178,55,162,183,68,124,75,28,223,217,227,118,150,239,
    83,248,250,196,122,64,124,130,84,107,117,69,155,189,18,155,
    253,188,123,217,18,155,206,47,226,91,171,188,35,62,42,139,
    50,241,203,93,190,70,60,250,252,69,51,168,110,75,183,43,
    115,237,100,153,95,6,45,7,249,199,107,169,120,90,203,116,
    223,115,55,164,94,115,125,220,8,211,202,105,210,54,134,156,
    167,217,92,177,143,53,145,223,255,30,98,241,13,200,241,167,
    36,198,101,40,235,94,132,195,242,152,135,187,118,79,8,132,
    19,245,225,128,84,74,15,51,68,64,199,87,150,241,27,156,
    103,244,78,49,162,23,84,244,102,56,63,153,71,112,211,1,
    194,20,5,60,66,100,204,241,169,124,102,124,44,159,201,143,
    152,252,162,110,66,204,24,133,76,126,108,92,164,127,151,16,
    234,5,99,169,152,23,255,6,184,200,116,146,
};

EmbeddedPython embedded_m5_internal_param_RubyMemoryControl(
    "m5/internal/param_RubyMemoryControl.py",
    "/mnt/hgfs/ShareShen/gem5-origin-stable-2015-9-3/build/x86/python/m5/internal/param_RubyMemoryControl.py",
    "m5.internal.param_RubyMemoryControl",
    data_m5_internal_param_RubyMemoryControl,
    2748,
    9498);

} // anonymous namespace
