#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_Bridge[] = {
    120,156,197,89,95,115,219,198,17,223,3,64,74,164,36,139,
    250,111,253,177,197,182,227,134,245,68,98,226,68,113,58,81,
    221,58,105,58,211,204,84,73,193,116,236,48,158,162,16,113,
    34,65,145,0,11,156,36,211,35,189,84,158,182,211,247,246,
    27,244,161,159,165,47,253,70,237,238,30,0,66,148,148,73,
    166,45,45,145,167,195,98,111,239,118,247,183,123,123,167,22,
    36,63,5,252,254,172,10,16,255,77,0,120,248,17,208,3,
    232,11,104,10,16,82,128,183,12,199,5,136,222,7,175,0,
    175,1,154,6,72,3,46,177,99,194,215,6,4,179,60,166,
    8,61,147,41,2,134,101,144,22,52,11,240,44,88,0,75,
    22,225,184,12,209,239,64,8,17,8,120,238,77,129,55,13,
    175,81,58,118,74,44,112,26,136,88,102,98,9,188,25,38,
    150,193,155,229,206,12,12,43,32,103,161,57,71,108,205,59,
    40,246,33,138,157,103,177,255,34,177,30,190,89,1,239,14,
    177,227,186,190,34,78,139,56,121,190,121,150,82,73,87,185,
    0,205,197,180,191,148,235,47,231,250,43,185,254,106,174,191,
    150,235,223,229,62,174,108,17,186,235,208,221,128,238,38,28,
    161,177,22,178,85,108,129,52,161,123,15,154,247,64,226,103,
    11,46,209,158,222,98,110,196,125,30,177,148,141,216,230,17,
    85,104,86,65,226,103,91,143,40,66,163,182,138,62,242,255,
    141,63,53,244,17,168,89,108,78,101,20,251,97,224,248,193,
    81,232,27,244,190,72,13,121,180,69,205,84,226,218,79,200,
    181,255,0,246,171,103,36,174,189,0,20,44,72,151,158,1,
    23,220,185,48,96,88,131,115,1,93,11,60,19,206,113,154,
    2,45,160,45,224,210,128,23,38,49,92,96,107,161,3,238,
    131,165,180,95,187,236,0,45,105,10,46,10,112,94,128,198,
    243,115,131,8,199,37,136,254,14,175,182,88,232,52,11,53,
    224,28,91,11,46,45,184,40,194,51,100,66,82,183,68,234,
    139,231,231,168,41,82,26,53,11,87,123,144,83,151,84,241,
    252,40,112,251,82,205,97,223,25,184,145,219,119,62,142,124,
    175,45,107,229,148,37,140,119,7,174,234,216,60,198,36,99,
    244,7,138,101,133,129,84,51,216,57,242,3,207,233,135,222,
    73,79,170,105,18,228,28,249,61,233,56,252,242,151,253,65,
    24,169,79,163,40,140,108,178,39,19,123,161,155,141,32,107,
    182,122,97,44,107,52,27,79,99,147,120,69,220,71,3,150,
    72,11,224,117,210,96,79,198,173,200,31,40,116,147,150,72,
    220,36,173,70,14,226,38,126,129,77,189,31,168,122,167,125,
    20,215,27,29,55,146,141,142,12,234,109,217,223,219,9,35,
    191,237,7,59,177,114,15,123,114,231,209,59,239,238,237,252,
    120,231,189,250,225,137,223,243,234,47,63,252,160,62,24,170,
    78,24,212,251,123,117,63,80,18,45,212,171,231,109,179,139,
    239,23,105,150,51,191,237,248,172,159,211,145,189,129,140,200,
    140,241,6,173,64,84,196,172,40,10,83,212,196,28,246,10,
    248,53,197,150,49,35,14,124,210,176,69,90,19,178,172,60,
    150,200,193,2,142,13,136,182,8,41,93,252,8,114,45,226,
    165,65,239,12,126,247,107,50,141,166,118,77,242,191,38,158,
    51,186,16,102,200,185,79,14,15,128,33,82,128,110,17,52,
    116,16,113,26,75,209,144,90,100,39,49,6,10,183,32,254,
    43,160,169,17,52,231,144,0,234,210,4,17,84,64,149,41,
    238,145,186,138,19,254,129,49,217,168,209,242,15,24,30,170,
    227,199,225,89,192,78,160,62,71,81,3,45,243,197,240,243,
    195,174,108,169,120,27,9,95,133,39,213,150,27,4,161,170,
    186,158,87,117,149,138,252,195,19,37,227,170,10,171,15,226,
    26,249,213,94,72,17,150,201,27,14,82,68,145,247,17,81,
    250,193,243,91,10,31,150,248,129,189,16,75,133,232,232,132,
    94,140,116,18,209,150,202,166,69,42,50,114,200,11,97,240,
    56,196,74,211,35,223,29,124,126,154,174,132,17,90,43,166,
    120,138,101,239,72,149,25,154,110,28,59,188,18,162,51,10,
    73,240,169,219,59,145,44,29,113,164,112,65,212,213,107,152,
    36,14,239,146,78,169,9,88,175,32,12,188,33,46,211,111,
    189,69,43,184,203,104,156,101,60,174,32,22,167,176,45,226,
    223,162,88,53,90,86,130,192,98,138,66,202,136,10,24,3,
    34,129,1,34,242,18,179,79,205,224,244,193,170,113,140,126,
    159,122,52,216,222,162,230,30,53,247,169,217,78,181,159,144,
    9,230,198,77,240,152,166,53,88,111,214,144,220,101,166,26,
    122,87,226,108,125,20,103,152,44,27,20,47,6,69,213,40,
    94,44,74,172,209,19,106,145,149,35,209,132,248,75,74,227,
    20,87,44,140,66,8,131,129,122,163,16,97,123,217,21,178,
    195,116,138,110,155,32,155,199,109,59,135,91,155,92,197,160,
    181,215,211,68,233,16,135,134,171,189,73,162,10,55,24,188,
    74,205,247,38,108,245,17,240,218,215,128,247,17,173,160,146,
    0,111,142,1,87,198,111,197,104,153,137,43,178,45,116,105,
    12,112,132,54,235,6,180,253,144,122,230,117,229,223,12,208,
    18,149,127,145,3,26,173,210,200,107,118,128,157,225,26,41,
    148,135,216,26,150,5,207,130,53,220,233,13,222,233,223,225,
    157,158,171,5,174,171,116,18,55,57,143,235,78,129,44,115,
    100,194,106,178,131,199,37,108,7,81,248,114,88,13,143,170,
    138,85,167,156,187,255,32,222,125,16,127,132,217,180,250,132,
    243,152,206,167,58,99,70,114,64,25,143,134,126,250,178,37,
    121,243,228,39,199,209,9,206,225,100,231,36,155,50,162,109,
    133,236,106,164,6,231,84,31,171,136,50,252,36,77,94,206,
    76,78,26,124,70,115,150,217,222,166,88,67,100,149,5,47,
    204,209,9,158,11,52,126,139,223,143,201,7,164,188,4,42,
    185,237,134,94,54,107,68,186,217,111,95,65,207,100,244,177,
    235,56,193,111,82,212,20,71,168,161,175,153,198,195,159,128,
    43,88,1,127,4,194,5,186,63,137,135,44,124,8,8,75,
    196,254,91,224,192,185,161,106,224,92,212,160,74,129,57,48,
    69,197,143,153,85,23,17,159,193,159,115,81,151,110,245,102,
    82,163,230,183,122,43,203,99,12,168,111,181,157,91,87,19,
    30,249,168,227,198,196,166,179,216,40,144,71,59,70,86,95,
    98,22,159,16,186,166,245,108,14,45,236,197,8,91,180,89,
    110,138,37,35,135,152,119,169,121,148,129,69,164,180,255,255,
    26,183,225,246,13,222,209,123,199,215,180,16,139,151,62,63,
    197,117,136,30,158,197,67,33,141,135,71,89,60,72,222,224,
    94,243,249,133,90,131,60,127,105,8,60,108,98,181,71,103,
    59,11,100,1,154,69,138,28,46,209,69,18,88,34,77,112,
    148,14,175,236,158,108,150,3,109,176,204,249,218,175,212,188,
    156,100,226,32,215,238,247,220,254,161,231,62,9,104,70,154,
    182,149,134,154,145,234,80,201,235,64,97,34,110,83,131,31,
    247,82,93,78,39,153,52,62,192,9,50,29,56,68,188,176,
    197,153,226,203,142,172,246,101,255,16,15,173,29,127,80,61,
    234,185,109,246,147,153,232,248,121,170,163,98,71,143,87,39,
    241,67,106,195,106,43,12,48,183,159,180,84,24,85,61,137,
    199,57,233,85,119,170,188,49,84,253,184,234,30,226,91,183,
    165,52,236,175,6,48,23,197,110,212,142,25,119,199,103,212,
    157,180,159,29,60,173,251,120,32,24,64,182,29,235,179,100,
    150,231,185,212,215,81,132,59,39,30,212,212,80,103,52,170,
    84,236,93,106,126,4,111,96,59,120,31,39,232,211,76,100,
    188,162,216,52,74,6,159,159,52,199,23,196,29,95,143,226,
    127,126,155,40,214,151,69,73,44,23,137,83,78,209,125,1,
    181,37,218,14,154,229,148,56,195,237,44,19,231,82,226,29,
    110,231,153,88,73,137,11,220,46,50,113,41,37,46,115,187,
    194,196,213,148,184,198,237,93,38,174,167,119,92,27,76,220,
    132,230,22,93,246,16,229,30,165,151,169,255,54,189,112,92,
    78,58,34,207,254,167,89,197,126,252,38,85,176,63,132,164,
    246,184,45,163,136,188,126,115,58,163,116,69,122,36,202,43,
    199,87,50,75,99,32,118,90,145,116,149,212,222,218,154,172,
    170,156,153,244,252,175,70,25,226,122,193,254,52,211,234,146,
    235,174,225,50,59,81,159,7,217,137,226,89,176,129,149,187,
    197,149,251,62,85,238,231,108,2,199,208,197,251,8,164,133,
    204,18,132,133,64,158,57,121,107,232,202,156,150,229,14,6,
    50,240,236,135,144,47,182,249,245,36,177,64,89,240,18,114,
    245,143,41,150,177,186,190,30,147,148,236,115,90,178,55,11,
    89,20,78,216,175,12,225,191,164,16,174,113,37,153,101,124,
    123,159,26,206,241,89,122,183,127,154,121,101,117,28,159,158,
    236,185,67,58,215,221,246,10,171,47,190,247,225,39,181,54,
    206,20,185,65,91,198,44,224,182,119,36,129,124,174,31,213,
    250,53,54,249,123,39,246,95,73,22,114,251,91,18,67,187,
    94,74,80,27,215,89,227,193,72,210,55,188,38,81,37,22,
    149,80,212,206,56,51,95,110,246,221,24,93,224,96,153,16,
    224,49,139,174,199,91,225,73,160,88,252,119,27,65,51,18,
    76,190,137,73,189,125,163,200,184,231,158,202,155,215,240,157,
    6,208,18,54,211,37,220,204,195,1,193,57,12,189,45,149,
    188,18,188,28,209,201,245,141,135,126,140,194,33,158,158,249,
    24,74,80,113,156,137,87,17,63,193,9,78,33,185,235,195,
    42,66,20,177,142,88,17,217,175,81,42,150,4,151,108,99,
    255,209,208,139,164,155,35,125,240,26,198,54,239,6,243,89,
    156,240,229,123,90,49,81,72,49,94,14,220,190,190,47,229,
    75,64,251,7,144,92,208,216,111,101,241,70,248,229,211,174,
    190,101,192,12,200,229,36,87,143,246,123,68,39,24,244,247,
    118,83,173,118,159,122,94,100,83,104,56,167,146,42,81,190,
    255,239,239,177,175,242,124,90,251,95,201,190,190,94,230,3,
    210,245,247,159,244,194,214,177,244,18,158,123,183,243,252,60,
    236,187,72,191,121,150,134,159,206,178,48,246,222,139,104,212,
    202,24,53,150,145,239,246,40,144,200,112,41,89,145,61,199,
    151,77,102,201,158,184,16,204,109,154,140,174,72,182,125,10,
    16,30,158,177,38,59,8,249,252,122,146,200,15,155,52,12,
    245,145,75,223,31,61,161,187,203,152,174,199,232,218,185,52,
    95,66,72,210,174,98,138,50,238,43,150,57,91,41,89,179,
    51,37,171,52,101,242,221,224,28,158,182,203,86,105,102,86,
    228,127,183,17,182,101,99,123,165,36,254,3,252,116,177,157,
};

EmbeddedPython embedded_m5_internal_param_Bridge(
    "m5/internal/param_Bridge.py",
    "/mnt/hgfs/ShareShen/gem5-origin-stable-2015-9-3/build/x86/python/m5/internal/param_Bridge.py",
    "m5.internal.param_Bridge",
    data_m5_internal_param_Bridge,
    2416,
    7510);

} // anonymous namespace
