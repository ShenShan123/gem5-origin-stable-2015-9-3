#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_BasePrefetcher[] = {
    120,156,205,89,95,115,219,198,17,223,3,64,74,164,36,139,
    250,111,91,178,5,199,113,195,122,42,49,113,162,56,157,184,
    158,58,105,50,147,204,68,113,192,116,236,48,153,162,16,113,
    36,65,145,0,11,156,44,211,149,94,42,79,219,153,62,247,
    35,244,161,95,163,15,125,238,55,106,119,247,0,10,212,31,
    59,51,237,152,149,200,211,97,177,183,183,187,247,219,189,189,
    83,19,210,159,2,126,127,105,3,36,255,20,0,62,126,4,
    244,0,250,2,26,2,132,20,224,47,195,126,1,226,15,192,
    47,192,75,128,134,1,210,128,19,236,152,240,189,1,225,44,
    143,41,66,207,100,138,128,97,25,164,5,141,2,60,9,23,
    192,146,69,216,47,67,252,91,16,66,132,2,158,250,83,224,
    79,195,75,148,142,157,18,11,156,6,34,150,153,88,2,127,
    134,137,101,240,103,185,51,3,195,10,200,89,104,204,17,91,
    227,10,138,189,139,98,231,89,236,191,72,172,143,111,86,192,
    191,66,236,168,215,119,196,105,17,39,207,55,207,82,42,153,
    150,11,208,88,204,250,75,185,254,114,174,191,146,235,175,230,
    250,107,185,254,213,92,255,90,174,127,61,215,95,207,245,55,
    184,143,150,44,66,247,6,116,111,66,119,19,90,232,220,133,
    145,214,54,72,19,186,183,160,113,11,36,126,108,56,65,255,
    251,139,185,17,111,241,136,165,209,136,219,60,226,109,104,188,
    13,18,63,183,245,136,34,212,171,171,184,166,193,191,241,167,
    138,107,10,106,22,155,103,50,78,130,40,116,131,176,21,5,
    6,189,47,82,67,8,104,82,51,149,66,225,83,130,194,223,
    129,113,224,27,41,20,142,1,5,11,178,165,103,192,49,119,
    142,13,24,86,225,72,64,215,2,223,132,35,156,166,64,10,
    180,5,156,24,240,131,73,12,199,216,90,184,96,55,193,82,
    26,7,93,94,48,45,105,10,142,11,112,84,128,250,211,35,
    131,8,251,37,136,255,6,47,54,88,232,52,11,53,224,8,
    91,11,78,44,56,46,194,19,100,66,82,183,68,230,139,167,
    71,104,41,82,234,85,11,181,221,205,153,75,166,248,65,28,
    122,125,169,86,176,239,14,188,216,235,187,159,120,137,124,28,
    203,150,84,205,142,140,171,229,140,53,74,182,7,158,234,56,
    60,214,36,167,244,7,138,101,70,161,84,51,216,105,5,161,
    239,246,35,255,160,39,213,52,9,116,91,65,79,186,46,191,
    252,162,63,136,98,245,89,28,71,177,67,126,101,98,47,242,
    70,35,200,171,205,94,148,200,42,205,198,211,56,36,94,17,
    119,107,192,18,73,1,214,151,6,251,50,105,198,193,64,225,
    114,105,137,196,77,210,170,180,80,220,36,104,56,212,250,161,
    170,117,218,173,164,86,239,120,177,172,119,100,88,107,203,254,
    206,86,20,7,237,32,220,74,148,183,215,147,91,247,222,125,
    111,103,235,231,91,239,215,246,14,130,158,95,123,254,209,135,
    181,193,80,117,162,176,214,223,169,5,161,146,232,169,94,237,
    34,31,109,35,223,34,205,118,24,180,221,128,237,116,59,178,
    55,144,241,28,81,175,147,38,162,34,102,69,81,152,162,42,
    230,176,87,192,175,41,54,140,25,177,27,144,165,77,178,158,
    144,102,229,177,69,11,46,96,223,128,120,131,144,211,197,143,
    160,165,70,252,212,233,157,193,239,190,33,23,105,106,215,36,
    60,104,226,17,163,13,97,135,156,15,8,0,33,48,100,10,
    208,45,130,134,18,34,80,99,43,30,82,139,236,36,198,64,
    225,22,36,127,5,116,57,130,232,8,82,128,157,152,32,194,
    10,168,50,229,13,164,174,226,132,127,96,140,214,171,164,254,
    46,195,68,117,130,36,58,12,121,49,168,207,81,85,71,207,
    60,30,126,189,215,149,77,149,108,34,225,187,232,192,110,122,
    97,24,41,219,243,125,219,83,42,14,246,14,148,76,108,21,
    217,119,146,42,173,175,179,144,33,109,36,111,56,200,144,69,
    40,64,100,233,7,63,104,42,124,88,226,7,94,133,68,42,
    68,73,39,242,19,164,147,136,182,84,14,41,169,200,201,17,
    43,194,32,114,137,149,166,71,190,43,248,252,40,211,132,145,
    90,45,102,184,74,100,175,165,202,12,81,47,73,92,214,132,
    232,140,70,18,252,204,235,29,72,150,142,120,82,168,16,117,
    181,14,147,192,227,85,178,45,115,5,219,23,70,161,63,68,
    117,131,230,59,164,201,85,70,229,44,227,114,5,49,57,133,
    109,17,255,22,197,170,209,180,82,36,22,51,52,82,166,84,
    192,88,16,41,28,16,153,39,152,149,170,6,167,21,54,145,
    99,246,45,234,209,96,103,131,154,27,212,220,164,102,51,243,
    194,27,118,197,220,89,87,220,167,233,13,182,159,45,165,229,
    51,51,75,253,177,184,187,118,26,119,152,76,235,20,63,6,
    69,217,105,252,88,148,120,227,135,212,34,43,71,166,9,201,
    183,148,230,41,206,88,24,133,20,6,7,245,78,67,134,253,
    230,84,200,31,211,25,218,29,130,112,30,199,237,28,142,29,
    90,50,6,177,115,45,75,160,46,113,104,248,58,235,36,170,
    112,129,227,109,106,110,77,200,251,167,64,108,159,3,226,199,
    164,73,37,5,226,28,3,176,140,223,138,209,52,211,37,25,
    109,181,75,103,0,72,232,179,46,64,223,79,168,103,158,119,
    194,100,129,151,154,254,121,14,120,164,173,145,183,112,23,59,
    195,53,50,44,15,185,53,44,35,158,132,107,88,25,24,92,
    25,188,203,149,1,87,23,92,183,233,36,111,114,158,215,157,
    2,121,168,101,194,106,186,227,39,37,108,7,113,244,124,104,
    71,45,91,177,11,40,39,63,184,147,108,223,73,62,198,108,
    107,63,228,60,167,243,173,206,168,177,28,80,70,164,161,159,
    61,111,74,222,100,249,201,117,117,2,116,57,25,186,233,230,
    141,232,163,10,130,87,131,29,207,91,65,162,98,218,1,38,
    225,250,242,200,245,100,201,151,52,119,153,253,110,138,53,68,
    90,89,176,130,174,222,8,184,176,227,183,248,253,132,214,130,
    156,32,129,74,123,167,174,213,103,203,200,70,231,103,99,104,
    122,179,118,57,53,156,232,215,25,138,138,167,40,162,175,153,
    197,201,159,128,43,96,1,127,4,194,9,194,33,141,147,81,
    88,17,48,150,136,253,55,192,1,117,65,149,193,185,170,78,
    149,5,115,96,10,75,238,51,171,46,58,190,132,63,231,162,
    49,43,13,204,180,198,205,151,6,214,40,207,49,192,126,212,
    246,111,141,39,68,90,171,142,151,16,155,206,114,167,1,126,
    186,179,140,234,82,204,242,111,24,109,211,122,86,151,20,252,
    225,20,107,180,185,174,139,37,35,135,160,247,168,185,55,2,
    143,200,104,111,78,215,77,184,188,48,112,245,94,243,61,41,
    100,177,9,243,83,92,21,141,139,25,197,75,33,139,151,123,
    163,120,145,188,49,190,228,115,17,181,6,33,226,196,16,120,
    232,197,170,145,206,152,22,200,2,52,138,20,89,92,242,139,
    52,240,68,150,8,41,109,142,237,186,236,166,93,237,192,17,
    40,244,122,83,243,124,18,9,134,150,252,65,207,235,239,249,
    222,195,152,102,166,233,155,89,40,26,153,45,149,188,45,20,
    70,226,50,115,248,113,39,179,233,217,36,146,203,135,56,209,
    200,22,14,37,63,106,114,70,249,182,35,237,190,236,239,225,
    225,184,19,12,236,86,207,107,243,186,153,169,173,95,103,182,
    42,94,248,179,85,78,114,151,218,200,110,70,33,238,9,7,
    77,21,197,182,47,241,184,40,125,123,203,230,13,197,14,18,
    219,219,195,183,94,83,233,176,24,15,116,46,182,189,184,157,
    112,93,189,127,72,221,73,173,187,235,6,97,64,7,14,24,
    109,231,250,204,58,218,31,56,104,116,148,225,206,139,7,65,
    53,212,25,144,42,30,103,155,154,159,194,4,183,145,15,112,
    162,223,209,140,228,204,162,88,55,74,134,90,62,23,231,143,
    105,116,114,62,218,255,241,99,162,93,95,110,33,131,44,66,
    119,138,219,105,218,69,26,165,140,88,230,118,134,137,179,25,
    113,142,219,43,76,156,207,136,21,110,23,152,184,152,17,151,
    184,93,102,226,74,70,92,229,118,141,137,87,179,140,115,141,
    137,215,161,177,158,93,206,109,80,246,41,254,183,217,135,195,
    117,82,129,250,251,255,105,210,113,238,255,63,152,226,124,4,
    105,9,115,89,194,25,171,152,31,233,132,163,215,9,11,157,
    225,50,155,173,15,104,108,182,120,18,94,199,210,217,226,210,
    249,1,149,206,71,92,94,187,134,174,158,79,151,151,207,77,
    124,139,68,7,150,80,30,186,23,197,130,46,145,9,56,222,
    96,32,67,223,185,11,249,170,151,95,79,194,139,148,86,94,
    66,174,240,48,197,50,150,185,231,209,77,89,52,103,53,163,
    184,48,194,243,198,4,65,240,151,12,4,213,27,99,169,212,
    121,64,77,101,44,111,234,133,178,47,73,88,110,20,186,190,
    167,60,58,117,189,158,9,171,32,125,189,169,159,95,57,32,
    192,237,235,181,82,153,41,39,149,158,95,57,160,31,224,121,
    234,117,82,153,41,39,149,158,95,57,32,150,158,255,90,169,
    204,148,147,74,207,234,214,43,6,28,198,129,146,44,246,245,
    92,36,119,90,203,101,130,218,184,108,72,50,212,14,120,37,
    3,137,227,242,126,152,48,222,21,21,8,190,236,73,156,234,
    162,65,138,48,147,94,163,248,18,43,139,104,136,91,54,31,
    251,240,185,231,186,19,219,125,127,129,19,189,128,244,86,13,
    119,95,81,20,43,167,191,70,169,88,18,92,248,156,249,255,
    131,86,245,14,225,255,27,106,56,99,206,143,194,129,239,199,
    179,98,131,130,134,207,235,187,94,95,95,101,242,189,156,115,
    27,210,59,18,231,157,81,68,209,101,18,31,44,245,1,31,
    115,31,87,100,92,128,57,239,103,41,177,191,179,157,25,182,
    173,13,171,15,19,37,251,124,61,223,223,225,213,200,243,200,
    240,160,239,126,37,251,81,60,252,42,242,245,226,231,223,63,
    242,253,216,241,194,182,116,159,73,42,8,25,80,99,12,105,
    53,168,101,100,92,246,133,170,140,243,158,211,69,51,225,75,
    125,3,205,103,160,243,239,63,237,69,205,125,233,167,60,55,
    46,231,249,85,212,247,144,126,241,44,245,32,155,101,225,204,
    123,63,166,81,43,103,168,137,140,3,175,23,188,208,23,219,
    25,153,239,0,47,82,139,202,147,49,10,215,114,172,203,56,
    208,24,248,177,108,7,184,72,49,139,27,27,150,238,103,4,
    197,203,67,57,47,98,82,209,162,79,90,250,154,233,33,93,
    121,242,69,25,221,90,151,230,75,162,104,208,94,103,138,50,
    238,118,150,57,91,41,89,179,51,37,171,52,101,242,85,226,
    28,30,190,203,86,105,102,86,92,244,187,137,113,86,54,54,
    23,75,226,63,13,110,251,74,
};

EmbeddedPython embedded_m5_internal_param_BasePrefetcher(
    "m5/internal/param_BasePrefetcher.py",
    "/mnt/hgfs/ShareShen/gem5-origin-stable-2015-9-3/build/x86/python/m5/internal/param_BasePrefetcher.py",
    "m5.internal.param_BasePrefetcher",
    data_m5_internal_param_BasePrefetcher,
    2408,
    7653);

} // anonymous namespace
