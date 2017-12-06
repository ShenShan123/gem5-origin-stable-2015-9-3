#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_Counter_vector[] = {
    120,156,205,92,123,136,36,71,25,255,186,231,177,55,115,187,
    183,123,183,123,187,247,216,220,77,30,151,76,162,119,155,215,
    229,97,194,105,140,49,36,226,197,244,70,46,217,68,59,189,
    211,181,187,189,59,211,61,233,238,189,187,137,123,74,220,211,
    248,68,197,7,42,34,226,3,9,8,130,32,8,130,32,8,
    130,32,4,4,65,16,4,69,16,4,255,19,4,65,253,126,
    95,117,207,99,103,78,114,55,123,211,119,187,243,93,77,77,
    77,213,247,251,234,87,95,85,125,93,181,53,74,254,21,248,
    245,174,10,81,116,183,65,228,242,175,65,117,162,134,65,75,
    6,25,202,32,119,134,54,10,20,222,79,110,129,46,19,45,
    153,164,76,218,230,68,142,94,52,201,31,151,239,20,169,158,
    147,28,131,90,101,82,121,90,42,208,57,127,63,229,85,145,
    54,202,20,190,76,134,97,248,6,61,239,142,145,187,135,46,
    115,237,156,40,73,133,123,8,153,101,201,44,145,187,87,50,
    203,228,142,75,98,47,181,166,72,141,211,210,4,138,45,237,
    227,106,239,226,106,39,165,218,55,81,173,203,159,28,36,119,
    31,138,179,94,47,160,100,30,37,165,189,73,169,101,138,92,
    169,101,133,241,236,111,23,220,79,42,71,235,7,104,233,0,
    41,254,221,79,219,12,217,61,144,22,156,110,23,156,150,130,
    51,180,52,67,138,127,167,117,193,34,45,86,103,217,122,222,
    127,249,95,149,173,71,241,56,139,243,42,140,188,192,183,61,
    127,37,240,76,124,94,132,128,173,107,16,99,137,209,31,135,
    209,223,32,177,184,107,38,70,191,68,92,177,1,155,214,77,
    186,36,137,75,38,181,170,180,101,208,122,158,220,28,109,113,
    51,5,40,176,106,208,182,73,47,229,80,224,18,203,60,155,
    230,24,229,99,109,241,117,49,141,174,105,140,46,21,104,171,
    64,139,207,111,153,200,216,40,81,248,3,122,117,94,42,221,
    35,149,154,180,197,50,79,219,121,186,84,164,115,92,136,179,
    214,75,128,111,60,191,197,72,57,103,177,154,103,109,207,118,
    193,5,20,215,11,125,167,161,226,73,78,219,143,7,155,126,
    172,66,251,188,170,197,65,88,45,167,133,130,232,84,211,137,
    215,44,249,86,14,230,104,52,99,169,45,240,85,188,151,19,
    43,158,239,218,141,192,221,172,171,120,15,170,178,87,188,186,
    178,109,249,240,169,70,51,8,227,39,194,48,8,45,88,84,
    50,235,129,211,254,6,236,89,171,7,145,170,162,53,105,198,
    66,245,49,74,175,52,165,70,40,32,154,226,203,174,138,106,
    161,215,140,185,163,116,141,40,141,218,170,232,34,17,209,135,
    89,44,52,252,120,97,109,117,37,90,88,92,115,66,181,184,
    166,252,133,85,213,56,125,50,8,189,85,207,63,25,197,206,
    114,93,157,188,247,238,123,78,159,124,248,228,125,11,203,155,
    94,221,93,184,248,208,3,11,205,86,188,22,248,11,141,211,
    11,30,76,226,59,245,133,94,235,156,226,18,7,208,206,5,
    111,213,246,4,161,189,166,234,77,21,78,32,247,8,116,48,
    166,140,113,163,104,228,140,170,49,193,169,2,191,114,198,188,
    185,215,56,235,1,99,13,184,193,174,124,55,159,208,201,6,
    109,152,20,206,131,45,235,252,107,160,123,153,51,139,248,204,
    148,207,158,133,113,116,238,122,14,28,208,153,91,194,48,166,
    26,151,124,20,157,238,147,208,164,64,235,69,210,244,97,214,
    105,62,133,45,72,46,142,106,76,174,60,79,209,87,137,141,
    205,196,217,162,132,84,219,57,50,252,41,138,203,24,149,156,
    59,203,13,126,92,120,185,88,133,250,103,133,32,241,154,23,
    5,23,124,233,6,164,101,36,45,178,101,62,208,122,102,121,
    157,237,21,29,231,140,23,130,205,74,205,241,253,32,174,56,
    174,91,113,226,56,244,150,55,99,21,85,226,160,114,34,170,
    162,103,173,253,41,199,218,245,181,154,41,167,208,255,204,41,
    253,198,245,106,49,191,153,150,55,210,11,145,138,153,31,107,
    129,27,113,62,170,88,85,177,5,37,99,24,57,16,69,132,
    62,54,138,162,121,46,183,143,223,63,150,106,34,28,173,22,
    83,70,69,170,190,18,151,133,156,78,20,217,162,9,242,133,
    135,168,248,188,83,223,84,82,59,51,41,102,133,144,212,58,
    140,150,137,135,128,42,53,130,32,243,3,223,109,177,162,94,
    237,14,232,112,72,248,56,46,140,60,200,108,28,99,89,228,
    255,139,198,172,89,203,39,28,44,166,60,132,95,140,73,88,
    96,36,68,96,78,110,179,15,170,154,226,68,4,156,140,211,
    91,144,194,151,173,121,136,155,32,142,65,28,79,241,143,204,
    8,19,59,141,240,32,26,54,5,185,96,68,151,229,82,140,
    110,207,88,59,220,25,107,236,52,23,49,102,76,140,172,206,
    152,201,195,193,134,103,32,185,168,140,198,28,69,207,193,157,
    99,108,73,101,24,70,60,32,144,234,12,19,177,152,53,5,
    75,236,73,25,110,129,182,221,220,93,237,226,174,133,206,18,
    226,90,135,83,119,105,163,132,166,172,117,20,85,21,6,152,
    188,2,113,243,200,237,222,33,223,106,31,249,30,129,14,83,
    9,249,38,132,116,101,126,77,153,181,92,210,25,237,201,116,
    122,7,233,192,184,252,0,198,221,142,84,174,31,126,86,100,
    75,64,191,183,139,108,208,211,236,198,118,150,19,173,57,64,
    234,166,217,28,47,17,206,249,115,60,235,155,50,235,223,45,
    179,190,172,28,100,245,163,157,121,78,252,185,78,20,96,155,
    149,28,205,38,179,121,84,98,217,12,131,139,173,74,176,82,
    137,5,60,124,239,163,39,162,83,39,162,71,216,171,86,206,
    136,63,211,126,85,123,206,80,53,225,249,240,213,39,46,214,
    148,76,163,242,206,182,181,163,179,197,233,217,201,244,204,140,
    59,8,203,154,169,201,197,229,71,113,8,79,63,90,163,151,
    219,70,7,134,167,209,106,89,44,158,51,230,152,93,101,67,
    84,179,181,171,151,229,154,124,202,175,119,163,23,0,95,17,
    150,198,214,162,86,92,48,1,157,245,246,30,6,141,10,145,
    181,192,77,124,48,101,78,177,195,28,188,114,233,168,120,157,
    100,69,107,208,39,9,220,96,10,36,163,162,61,136,64,134,
    233,74,162,242,150,49,104,5,33,62,105,17,171,6,41,193,
    174,42,122,80,138,234,5,197,211,244,169,174,177,151,78,251,
    185,100,205,218,61,237,231,219,254,76,72,245,150,166,246,124,
    175,227,67,47,173,57,17,138,105,111,214,25,206,157,185,163,
    189,218,100,111,62,50,134,237,209,237,217,80,237,165,14,191,
    48,113,30,53,166,205,46,214,220,3,113,111,155,48,70,154,
    55,10,45,143,211,149,167,123,91,207,35,47,66,149,188,40,
    63,57,38,171,28,189,30,123,138,107,114,184,154,246,232,40,
    164,163,227,47,237,209,161,100,210,187,44,123,27,72,19,44,
    216,54,13,222,34,242,42,16,59,178,60,169,2,45,21,73,
    141,97,11,130,141,95,33,221,248,21,147,141,95,103,175,56,
    46,233,146,164,39,100,175,72,216,224,37,123,197,201,116,175,
    200,187,188,9,73,236,79,182,131,188,177,75,54,128,211,216,
    0,34,49,147,108,0,151,120,203,120,64,18,179,201,78,111,
    105,14,123,93,36,14,97,67,137,196,97,114,103,37,113,132,
    220,57,73,28,197,192,199,116,37,227,44,125,137,135,134,63,
    239,89,2,72,143,158,213,125,221,102,174,38,37,196,197,209,
    122,62,240,242,209,186,211,88,118,157,51,171,104,19,13,215,
    82,79,97,166,40,166,186,81,96,148,27,87,2,34,111,79,
    167,104,206,143,214,235,61,192,77,180,81,200,24,119,131,154,
    184,186,231,214,84,165,161,26,203,188,11,95,243,154,149,149,
    186,179,42,125,149,75,80,62,147,162,140,133,157,59,151,89,
    209,93,144,65,165,22,248,60,65,109,162,189,138,171,120,119,
    170,220,202,201,138,204,110,21,47,170,56,203,252,169,83,139,
    245,168,237,245,64,178,194,119,194,213,72,22,243,27,23,144,
    28,125,95,219,182,231,123,188,191,241,168,119,85,209,195,89,
    104,234,118,168,42,96,10,109,103,52,79,163,158,204,208,173,
    141,54,57,83,85,219,228,156,208,221,182,110,164,11,232,110,
    102,202,54,126,166,207,77,217,178,197,202,10,16,252,108,208,
    233,1,189,107,111,79,207,221,131,174,141,75,79,173,253,208,
    166,251,161,121,126,45,236,90,85,225,27,254,104,153,6,2,
    65,139,176,139,101,187,6,207,85,221,240,172,51,163,238,61,
    65,7,37,54,135,70,55,55,0,157,199,218,249,53,213,133,
    240,244,200,17,194,83,164,138,92,28,26,229,128,209,167,94,
    217,116,234,153,66,132,63,19,45,62,50,192,19,94,133,107,
    25,64,208,90,208,108,101,228,89,132,155,104,255,163,187,142,
    202,87,23,227,44,81,161,253,215,134,67,53,96,184,217,130,
    203,182,179,66,150,4,32,69,135,203,187,142,174,25,170,243,
    94,176,25,101,137,46,213,225,245,161,253,200,108,63,64,199,
    61,191,195,89,142,126,58,192,166,51,209,227,51,67,99,60,
    56,136,162,234,21,38,104,150,206,178,40,44,133,26,159,191,
    62,8,125,117,67,32,132,26,95,28,26,225,64,55,227,57,
    174,219,131,113,244,68,77,214,255,162,200,151,175,19,202,104,
    115,249,198,64,41,138,124,237,122,248,28,219,206,190,43,37,
    28,169,245,248,6,81,95,124,24,24,159,28,132,241,115,131,
    38,142,129,24,119,118,228,195,25,97,20,61,190,69,253,51,
    99,207,110,117,171,179,91,21,29,51,157,205,61,100,218,223,
    238,232,92,21,11,182,195,193,18,53,211,97,182,102,24,52,
    85,24,183,116,216,19,15,53,172,83,16,119,246,184,75,87,
    213,85,172,236,222,62,138,167,168,253,148,199,85,81,28,6,
    45,219,78,140,198,95,176,109,217,101,90,239,132,120,12,226,
    113,136,39,32,158,132,120,10,226,125,16,239,135,120,6,226,
    89,136,69,8,196,142,173,115,16,47,64,32,234,103,189,212,
    99,207,145,109,150,239,231,38,86,208,22,108,87,52,142,154,
    37,179,104,148,140,146,89,202,141,243,79,233,74,63,166,216,
    89,215,147,30,65,232,143,78,186,198,91,136,78,234,163,43,
    73,140,178,152,6,37,199,210,160,164,156,85,65,162,36,161,
    73,29,175,44,165,241,74,29,151,28,79,227,146,19,105,92,
    114,95,26,151,156,76,227,146,83,105,92,114,127,26,151,60,
    144,198,37,167,211,184,228,76,26,151,60,152,198,37,103,211,
    184,228,92,26,151,60,148,198,37,15,147,123,40,141,84,30,
    78,34,149,238,17,73,204,147,123,84,18,55,145,59,47,137,
    99,228,222,36,137,227,228,30,147,68,133,220,227,146,184,153,
    220,138,36,110,33,247,102,73,220,74,238,45,146,184,141,220,
    91,37,113,130,220,219,36,113,59,169,59,104,189,74,75,119,
    146,123,66,114,238,66,120,20,143,215,134,10,143,142,126,85,
    32,129,167,239,211,110,70,69,173,7,179,5,97,61,68,201,
    195,159,43,69,68,175,97,215,209,59,198,196,255,193,67,101,
    233,133,83,29,126,72,255,103,230,40,183,251,111,123,71,180,
    179,70,153,77,34,226,102,223,24,160,246,85,116,202,145,254,
    78,177,241,40,231,85,21,6,217,237,117,245,121,131,182,26,
    63,26,14,227,0,226,217,246,114,16,212,179,222,204,107,29,
    126,60,28,186,217,65,232,234,202,207,14,156,94,89,136,10,
    63,233,194,214,253,164,88,176,77,83,239,146,83,63,245,237,
    71,120,116,16,194,85,21,71,117,175,134,221,96,231,193,137,
    33,227,25,114,125,180,144,199,41,57,32,147,232,244,211,29,
    125,122,245,75,237,129,160,163,14,232,44,215,219,26,109,71,
    153,159,93,183,94,230,229,105,95,47,91,27,16,245,140,80,
    119,52,250,249,208,125,60,208,249,114,3,60,37,53,50,238,
    226,189,41,88,173,203,47,174,15,86,30,48,55,12,214,182,
    46,191,188,62,88,163,27,8,107,91,151,95,209,80,115,207,
    129,126,160,205,160,153,213,188,131,78,226,230,127,221,133,233,
    218,3,143,59,96,57,205,166,242,221,204,3,143,90,141,223,
    12,215,107,51,253,240,84,163,25,103,246,36,74,30,176,65,
    129,223,14,135,107,186,31,87,228,189,154,213,163,123,125,126,
    155,219,127,115,104,62,14,2,118,193,105,118,177,113,228,123,
    69,141,142,149,248,221,174,115,113,89,177,122,89,114,81,20,
    248,253,174,123,70,248,143,12,61,35,55,255,135,225,48,13,
    112,139,97,166,157,5,143,168,53,248,227,174,123,142,48,195,
    238,194,216,66,251,127,218,245,177,85,171,43,39,179,104,139,
    190,100,197,10,252,121,56,92,243,253,184,86,113,96,182,94,
    15,106,89,70,147,160,121,143,34,127,29,14,231,128,184,5,
    47,111,236,101,167,182,145,233,99,250,68,135,191,237,64,119,
    245,235,228,65,11,145,208,137,84,166,43,100,89,141,64,139,
    191,239,192,151,94,21,17,124,239,233,224,99,112,250,146,229,
    140,132,8,211,75,59,184,226,121,206,63,66,121,30,206,184,
    90,241,78,92,173,216,146,195,239,182,169,111,87,116,66,137,
    5,234,158,56,124,117,193,238,181,139,14,16,227,0,146,133,
    243,165,93,155,97,24,72,62,29,109,232,17,15,160,254,65,
    148,158,78,159,52,114,198,140,49,113,173,11,156,67,3,152,
    190,25,173,105,170,103,185,230,150,203,45,169,38,255,220,117,
    119,188,18,6,126,102,71,165,192,57,81,224,95,195,225,26,
    48,121,102,233,162,48,121,162,253,127,119,161,186,230,160,212,
    160,157,96,20,121,171,126,215,8,60,147,13,53,101,59,40,
    186,252,135,134,117,196,131,86,118,74,246,78,89,122,98,89,
    222,137,26,134,113,29,16,122,126,164,194,56,115,132,90,141,
    124,23,194,107,243,161,3,158,3,176,241,84,120,62,251,51,
    111,137,30,99,198,110,47,135,106,78,211,169,121,217,69,46,
    176,28,74,117,40,15,64,215,115,242,228,70,187,39,49,97,
    36,79,122,171,111,163,238,67,39,214,135,32,228,152,73,231,
    132,9,30,115,202,243,70,75,65,224,234,140,133,155,33,22,
    110,91,88,184,161,96,189,2,129,10,45,28,119,183,46,64,
    180,40,93,182,92,130,248,24,196,107,16,219,16,159,128,192,
    105,79,235,211,16,159,133,192,177,65,235,11,16,95,130,144,
    131,46,95,129,192,9,45,235,235,16,223,132,192,65,31,11,
    39,103,172,239,64,124,183,103,164,39,199,96,118,172,161,108,
    148,121,185,199,214,35,51,184,195,77,124,15,109,225,202,104,
    209,56,106,20,77,28,75,185,170,159,177,157,199,86,74,134,
    76,67,59,254,72,136,6,6,51,232,187,139,173,200,66,142,
    53,217,54,145,62,108,155,156,39,66,111,203,58,231,172,211,
    208,127,126,64,110,212,91,183,66,224,40,134,117,71,155,10,
    184,12,46,23,70,245,101,93,94,167,202,133,38,185,191,100,
    221,7,129,35,56,242,40,103,199,49,49,28,62,10,121,167,
    30,161,35,156,180,80,95,80,171,83,104,244,125,164,111,184,
    233,27,199,103,16,39,143,112,2,2,127,174,160,52,89,226,
    254,194,159,206,200,25,101,94,229,230,115,227,83,165,252,248,
    222,82,190,52,150,147,251,228,19,198,180,89,206,151,246,206,
    190,163,100,148,205,217,141,146,241,63,97,121,60,162,
};

EmbeddedPython embedded_m5_internal_Counter_vector(
    "m5/internal/Counter_vector.py",
    "/mnt/hgfs/ShareShen/gem5-origin-stable-2015-9-3/build/x86/python/m5/internal/Counter_vector.py",
    "m5.internal.Counter_vector",
    data_m5_internal_Counter_vector,
    3438,
    18208);

} // anonymous namespace
