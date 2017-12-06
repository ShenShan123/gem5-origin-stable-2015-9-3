#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_X86KvmCPU[] = {
    120,156,205,89,109,111,27,199,17,158,187,35,41,145,146,44,
    217,178,36,191,200,22,157,196,49,227,70,162,95,34,219,69,
    92,183,178,226,180,74,98,69,61,58,177,163,4,189,158,238,
    86,228,73,228,29,113,183,148,77,67,6,218,202,72,138,2,
    5,10,20,237,151,22,232,135,162,31,218,95,211,127,212,206,
    204,222,145,39,82,10,82,40,32,107,145,235,225,222,236,238,
    204,236,51,47,187,231,64,252,47,139,223,159,20,1,162,151,
    58,128,139,31,13,234,0,13,13,54,53,208,132,6,238,89,
    216,205,66,248,30,184,89,120,13,176,169,131,208,225,0,9,
    3,190,212,193,31,231,49,57,168,27,220,163,65,187,0,34,
    3,155,89,120,234,159,134,140,200,193,110,1,194,95,130,166,
    105,190,6,207,220,17,112,71,225,53,206,142,68,158,39,28,
    5,234,44,112,103,30,220,49,238,44,128,59,206,196,24,180,
    167,64,140,195,230,4,177,109,158,194,105,175,227,180,147,60,
    237,191,105,90,23,159,204,128,123,138,216,81,174,47,136,51,
    67,156,188,222,36,207,50,149,72,121,26,54,207,36,244,116,
    138,62,155,162,103,82,244,108,138,158,75,209,231,82,244,249,
    20,125,33,69,95,76,209,243,41,250,82,138,190,156,162,23,
    82,116,49,69,95,73,209,111,164,232,55,83,244,91,41,250,
    106,138,126,59,69,95,75,209,165,20,253,78,138,190,158,162,
    127,192,52,90,255,12,236,188,11,59,139,176,179,4,219,8,
    136,211,29,75,151,65,24,176,115,3,54,111,128,192,79,25,
    14,16,51,238,153,212,136,155,60,98,186,51,226,22,143,184,
    13,155,183,65,224,231,150,26,145,131,74,105,22,113,232,253,
    7,255,149,52,164,228,56,54,123,34,140,188,192,183,60,127,
    59,240,116,122,158,163,134,80,235,80,51,18,195,119,149,224,
    251,79,96,236,186,122,12,223,87,128,19,107,164,75,93,135,
    87,76,188,210,161,93,130,125,13,118,50,224,26,176,143,203,
    100,73,128,170,6,7,58,124,101,16,195,43,108,51,8,178,
    203,144,145,10,187,59,12,50,53,211,8,188,202,194,126,22,
    42,207,246,117,234,216,205,67,248,15,120,57,207,147,142,242,
    164,58,236,99,155,129,131,12,188,202,193,83,100,194,174,157,
    60,169,175,61,219,71,77,177,167,82,202,160,180,235,41,117,
    73,21,215,11,125,187,33,228,20,210,86,211,14,237,134,245,
    236,222,157,143,247,26,171,27,159,149,10,9,87,16,45,53,
    109,89,51,121,152,65,246,104,52,37,79,23,248,66,142,33,
    177,237,249,174,213,8,220,86,93,200,81,154,203,218,246,234,
    194,178,248,225,90,163,25,132,242,81,24,6,161,73,38,229,
    206,122,96,119,70,144,65,157,122,16,137,18,173,198,203,152,
    52,189,36,238,237,38,207,72,2,176,168,52,216,21,145,19,
    122,77,137,59,165,102,36,110,154,173,68,123,196,77,100,97,
    83,110,248,178,92,171,110,71,229,74,205,14,69,165,38,252,
    114,85,52,150,23,131,208,171,122,254,98,36,237,173,186,88,
    188,117,227,230,242,226,15,23,111,151,183,90,94,221,45,191,
    184,119,167,220,108,203,90,224,151,27,203,101,207,151,2,141,
    84,47,247,152,103,9,89,206,208,66,207,189,170,229,177,138,
    86,77,212,155,34,156,160,222,11,36,132,54,165,141,107,57,
    205,208,74,218,4,82,89,252,26,218,188,62,166,173,123,164,
    164,67,138,19,190,50,105,68,209,54,107,176,171,67,56,79,
    120,217,193,143,70,27,140,168,169,208,51,157,159,253,156,172,
    163,122,119,12,66,129,234,220,103,140,33,216,144,243,62,109,
    187,15,12,148,44,236,228,64,1,8,113,167,16,21,182,169,
    69,118,154,70,199,201,51,16,253,9,208,218,8,157,125,136,
    97,117,96,128,230,79,129,44,80,132,195,222,89,92,240,55,
    140,204,74,137,196,95,103,132,200,154,23,5,207,125,222,7,
    162,217,151,42,104,153,141,246,167,91,59,194,145,209,2,118,
    124,17,180,138,142,237,251,129,44,218,174,91,180,165,12,189,
    173,150,20,81,81,6,197,171,81,137,182,214,60,157,128,172,
    51,95,187,153,128,138,0,128,160,82,63,92,207,145,248,99,
    154,127,240,46,68,66,34,64,106,129,27,97,63,77,81,21,
    210,36,33,37,25,57,96,65,24,63,22,177,210,242,200,119,
    10,127,175,36,146,48,72,75,185,4,82,145,168,111,203,2,
    163,211,142,34,139,37,161,126,6,34,77,188,103,215,91,130,
    103,71,40,73,20,136,72,37,195,128,161,120,142,212,74,172,
    192,170,249,129,239,182,81,82,207,185,70,66,156,99,64,142,
    51,36,103,16,142,35,216,230,240,255,156,54,171,59,153,24,
    132,185,4,136,20,26,37,48,12,180,24,9,8,202,3,12,
    67,37,157,227,8,107,199,158,250,6,81,52,216,156,167,230,
    18,53,151,169,89,72,12,48,56,43,76,244,90,225,46,173,
    172,179,234,172,36,109,154,145,40,233,30,242,182,243,93,111,
    195,192,89,33,175,209,201,183,186,94,147,161,32,27,62,160,
    22,89,217,31,13,136,158,80,72,39,239,226,201,200,145,208,
    37,136,234,58,10,155,204,164,32,91,26,77,48,110,18,112,
    211,232,173,166,208,107,210,110,49,116,205,243,73,196,180,136,
    67,129,214,188,72,83,101,143,176,121,145,154,43,131,55,124,
    23,126,213,62,248,189,79,66,76,197,240,155,96,216,21,240,
    59,165,59,70,188,27,157,140,58,221,3,59,194,92,230,8,
    204,189,77,148,209,175,255,208,224,22,107,253,97,10,110,36,
    168,158,86,110,29,137,246,28,233,148,6,218,28,22,10,79,
    253,57,204,253,58,231,254,27,156,251,185,126,224,106,82,5,
    116,131,99,186,34,178,100,156,109,3,102,227,156,30,229,177,
    109,134,193,139,118,49,216,46,74,214,158,226,239,253,171,209,
    210,213,232,125,140,172,197,7,28,211,84,108,85,209,51,20,
    77,138,126,52,244,209,11,71,112,46,229,95,150,165,130,157,
    197,129,207,138,115,52,98,110,134,76,171,39,54,231,176,31,
    201,144,162,253,128,173,94,232,88,157,148,248,136,150,45,176,
    201,13,109,14,241,85,208,88,54,75,197,123,174,218,248,41,
    126,31,210,54,144,254,2,232,172,97,86,148,228,172,20,169,
    103,190,123,8,67,3,83,201,44,227,26,159,37,216,201,117,
    177,67,95,35,113,140,111,128,43,91,13,190,6,66,7,130,
    32,118,140,142,31,17,28,166,137,253,23,192,30,116,68,29,
    193,113,169,66,181,3,115,96,184,138,238,50,171,42,43,62,
    130,223,166,220,47,73,254,70,92,187,166,147,127,166,19,211,
    24,86,223,41,193,103,14,7,63,218,166,154,29,17,155,138,
    104,93,143,238,38,144,78,209,137,17,125,112,24,27,85,11,
    90,36,219,87,93,132,81,250,188,168,77,235,41,220,220,164,
    230,86,7,50,90,210,55,16,49,23,224,248,172,111,169,108,
    242,37,201,146,97,233,39,71,216,195,59,51,116,124,35,155,
    248,70,173,227,27,130,179,222,107,62,224,80,171,19,4,14,
    116,13,79,220,88,8,210,1,55,3,34,11,155,185,228,144,
    62,66,7,184,206,225,154,136,60,159,187,213,201,58,31,159,
    172,233,208,141,196,56,249,30,37,53,70,122,242,229,40,73,
    49,245,80,34,102,147,174,43,99,119,176,163,96,65,205,139,
    1,71,31,66,198,253,186,221,216,114,237,7,191,166,69,105,
    101,39,113,86,61,81,99,42,173,6,57,154,118,156,38,252,
    115,57,81,103,111,192,145,231,14,174,209,81,131,253,204,13,
    28,14,55,79,106,162,216,16,141,45,60,17,215,188,102,113,
    187,110,87,121,183,140,88,205,79,19,53,37,131,164,183,220,
    137,174,83,27,20,157,192,199,52,209,114,100,16,22,93,129,
    7,69,225,22,23,139,156,99,138,94,84,180,183,240,169,237,
    72,229,56,135,163,0,215,218,118,88,141,184,172,222,125,78,
    228,16,118,219,178,60,223,195,163,198,1,28,78,238,73,144,
    238,192,86,170,205,134,190,205,230,51,234,76,218,245,44,183,
    213,104,126,216,108,153,2,85,210,18,133,7,172,28,31,165,
    187,114,124,243,125,235,183,230,203,255,11,253,98,57,126,119,
    50,253,102,251,244,171,52,133,51,84,5,199,99,5,19,65,
    126,127,50,13,207,244,105,248,108,213,28,158,118,163,177,118,
    36,196,31,78,166,217,116,191,102,21,123,79,12,77,181,124,
    162,26,73,241,199,147,233,118,174,79,183,207,87,155,173,71,
    123,194,151,195,219,187,83,177,130,93,81,254,220,213,178,68,
    74,117,11,110,102,86,85,12,158,98,154,34,148,109,85,87,
    210,193,209,92,162,230,29,106,126,68,205,143,169,89,161,102,
    149,154,71,212,252,20,134,83,181,191,135,107,252,138,22,35,
    199,201,105,23,245,188,158,211,243,70,242,39,39,211,59,179,
    65,227,163,254,146,107,11,190,67,201,165,94,111,196,133,87,
    142,56,197,8,221,254,82,155,167,34,158,106,172,108,92,99,
    81,167,122,123,161,10,178,83,84,108,81,173,125,162,98,139,
    75,148,33,20,39,127,133,239,179,198,50,239,14,89,11,243,
    30,196,199,185,227,234,171,147,164,37,5,50,203,9,133,45,
    135,23,221,168,90,83,34,252,173,39,180,29,186,15,89,233,
    232,118,192,7,218,246,89,222,77,117,233,198,187,169,61,245,
    47,64,6,203,64,186,24,185,79,23,35,251,108,8,75,87,
    119,35,93,192,102,33,29,234,125,241,220,234,177,137,186,251,
    32,201,236,102,83,248,174,121,189,131,139,249,4,43,3,198,
    5,133,183,191,67,234,108,105,104,103,181,9,173,223,75,169,
    12,78,233,202,219,154,237,248,229,224,55,152,17,253,175,4,
    209,37,10,115,221,104,110,254,140,26,142,223,221,208,253,113,
    103,123,230,143,128,107,43,18,156,8,233,22,237,219,25,240,
    148,203,101,65,210,193,54,100,23,112,69,93,72,209,187,235,
    234,197,150,202,45,174,192,147,70,208,182,44,117,61,132,191,
    235,150,53,140,180,177,134,107,252,133,22,163,188,135,105,67,
    195,164,161,207,232,249,92,94,227,179,79,207,123,71,37,29,
    229,26,117,13,210,142,76,14,36,147,29,155,242,251,177,36,
    115,146,249,185,190,88,183,27,234,125,6,223,208,155,111,66,
    124,111,106,94,235,236,13,221,45,243,221,147,186,249,67,183,
    225,115,25,31,195,204,219,201,118,53,150,151,18,117,150,148,
    58,15,237,72,40,125,248,21,93,99,89,206,29,201,135,60,
    159,63,238,155,99,197,117,67,211,246,171,194,218,19,116,44,
    228,10,166,127,112,165,29,73,209,144,23,123,30,10,191,213,
    176,30,139,70,16,182,31,7,174,144,87,122,103,143,15,148,
    138,37,89,162,120,228,18,135,121,251,150,82,76,248,80,189,
    195,226,219,150,254,231,171,245,192,217,21,110,204,115,233,120,
    158,15,130,134,141,253,71,175,82,241,146,85,78,247,60,119,
    67,26,53,211,211,27,137,208,179,235,222,75,33,207,31,187,
    69,180,63,71,219,22,241,248,228,147,135,242,173,227,30,110,
    216,116,161,141,40,127,106,215,119,17,86,199,175,65,211,92,
    62,110,154,79,2,199,174,175,52,61,231,24,251,227,4,158,
    179,225,5,31,136,61,207,17,199,88,166,251,188,87,10,92,
    97,173,178,242,237,40,82,60,242,66,207,195,213,160,69,100,
    50,246,104,152,175,249,145,124,130,248,64,3,244,142,223,8,
    3,71,68,81,50,254,104,251,196,76,236,159,201,35,142,72,
    125,94,68,25,160,251,147,11,203,158,243,38,133,176,80,84,
    61,244,137,144,231,232,178,199,9,110,237,184,240,154,30,58,
    132,120,167,110,204,212,107,132,7,116,36,143,234,216,208,107,
    200,252,100,30,99,31,229,61,67,43,96,230,203,24,227,83,
    249,204,248,88,62,147,31,49,248,45,209,132,54,173,23,50,
    249,177,113,237,127,255,91,56,151,215,10,250,194,84,94,251,
    47,80,27,69,129,
};

EmbeddedPython embedded_m5_internal_param_X86KvmCPU(
    "m5/internal/param_X86KvmCPU.py",
    "/mnt/hgfs/ShareShen/gem5-origin-stable-2015-9-3/build/x86/python/m5/internal/param_X86KvmCPU.py",
    "m5.internal.param_X86KvmCPU",
    data_m5_internal_param_X86KvmCPU,
    2677,
    9337);

} // anonymous namespace
