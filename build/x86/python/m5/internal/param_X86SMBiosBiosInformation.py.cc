#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_X86SMBiosBiosInformation[] = {
    120,156,205,89,81,83,27,201,17,238,217,93,9,36,192,128,
    49,96,27,12,242,221,249,44,115,7,186,243,25,219,137,29,
    39,62,151,83,241,85,130,47,171,75,217,167,187,202,102,209,
    14,98,101,105,87,217,29,48,186,192,75,112,37,185,183,188,
    228,15,164,42,15,249,55,249,71,73,119,143,118,89,4,24,
    108,167,74,6,52,140,122,103,122,186,123,190,238,233,158,173,
    67,239,39,135,159,95,148,0,226,127,10,0,15,255,4,180,
    0,218,2,106,2,132,20,224,93,128,23,57,136,110,129,151,
    131,87,0,53,3,164,1,251,216,49,225,59,3,130,81,158,
    147,135,150,201,20,1,221,34,72,11,106,57,120,22,76,130,
    37,243,240,162,8,209,31,64,8,17,8,120,238,13,129,55,
    12,175,144,59,118,10,204,112,24,136,88,100,98,1,188,17,
    38,22,193,27,229,206,8,116,39,64,142,66,109,140,134,213,
    206,33,219,37,100,59,206,108,255,67,108,61,124,50,13,222,
    57,26,142,114,125,75,35,45,26,201,235,141,51,151,137,68,
    202,73,168,157,79,250,83,153,254,133,76,127,58,211,159,201,
    244,103,51,253,139,153,254,37,238,163,148,231,161,121,25,154,
    115,208,156,135,13,52,220,100,42,209,21,144,38,52,23,160,
    182,0,18,255,174,192,62,218,214,59,159,153,177,200,51,166,
    210,25,37,158,113,21,106,87,65,226,95,73,207,200,67,181,
    60,131,251,229,255,23,127,202,184,95,160,70,177,217,150,81,
    236,135,129,227,7,27,161,111,208,243,60,53,180,187,117,106,
    134,122,219,252,136,182,249,223,192,123,236,25,189,109,222,3,
    100,44,72,151,150,1,123,220,217,51,160,91,134,93,1,77,
    11,60,19,118,113,153,28,9,208,16,176,111,192,247,38,13,
    216,195,214,194,205,88,0,75,233,61,110,242,102,104,78,67,
    176,151,131,221,28,84,159,239,26,68,120,81,128,232,95,240,
    195,60,51,29,102,166,6,236,98,107,193,190,5,123,121,120,
    134,131,144,212,44,144,250,226,249,46,106,138,148,106,217,66,
    105,215,50,234,146,42,158,31,5,110,91,170,69,236,59,29,
    55,114,219,206,243,187,183,171,191,249,210,15,99,250,60,65,
    51,68,109,87,161,73,202,197,100,82,24,175,116,92,181,105,
    51,23,147,204,211,238,40,230,30,6,82,141,96,103,195,15,
    60,167,29,122,91,45,169,134,137,181,179,225,183,164,227,240,
    195,39,237,78,24,169,199,81,20,70,54,89,152,137,173,208,
    77,103,144,125,235,173,48,150,101,90,141,151,177,137,189,162,
    209,27,29,230,72,2,176,228,52,217,147,113,61,242,59,36,
    165,230,72,163,137,91,153,182,140,155,56,192,166,210,14,84,
    101,179,177,17,87,170,155,110,36,171,155,50,168,52,100,123,
    117,57,140,252,134,31,44,199,202,93,111,201,229,155,159,125,
    190,186,252,147,229,47,42,235,91,126,203,171,236,220,189,93,
    233,116,213,102,24,84,218,171,21,63,80,18,109,214,170,188,
    222,90,43,56,227,60,173,251,210,111,56,62,107,236,108,202,
    86,71,70,99,68,189,76,50,137,9,49,42,242,194,20,101,
    49,134,189,28,126,76,49,111,140,136,53,159,116,174,147,29,
    8,125,86,22,111,4,2,1,47,12,136,230,9,77,77,252,
    19,180,253,136,169,42,61,51,248,217,111,201,88,154,218,52,
    9,35,154,184,203,8,68,40,226,200,251,4,10,180,10,193,
    40,7,205,60,104,120,33,42,53,222,162,46,181,56,156,216,
    24,200,220,130,248,31,128,198,71,96,237,66,15,116,251,38,
    136,96,2,84,145,226,4,82,103,112,193,63,51,110,171,101,
    18,127,141,1,163,54,253,56,124,25,240,182,80,159,61,173,
    138,150,249,186,251,116,189,41,235,42,38,252,125,27,110,149,
    234,110,16,132,170,228,122,94,201,85,42,242,215,183,148,140,
    75,42,44,93,139,203,180,211,246,100,130,185,148,95,183,147,
    96,140,240,128,24,211,95,60,191,174,240,203,20,127,225,93,
    136,165,66,188,108,134,94,140,116,98,209,144,202,38,33,21,
    25,57,100,65,24,78,14,13,165,229,113,220,57,252,254,48,
    145,132,49,91,206,39,8,139,101,107,67,21,25,172,110,28,
    59,44,9,209,25,151,196,120,219,109,109,73,230,142,200,82,
    40,16,117,181,12,131,69,230,69,210,50,49,10,107,26,132,
    129,215,69,193,253,250,117,146,233,34,227,115,148,17,58,141,
    232,28,194,54,143,255,243,98,198,168,91,61,76,230,19,92,
    82,28,85,192,168,16,61,96,32,70,247,49,102,149,13,14,
    58,172,44,251,241,7,212,163,201,246,60,53,87,168,89,160,
    102,49,177,199,192,140,50,214,111,148,59,36,136,193,150,96,
    157,105,75,205,68,103,239,144,47,94,58,240,69,12,186,85,
    242,41,131,60,239,192,167,44,10,208,209,3,106,113,40,123,
    171,9,241,55,116,28,144,239,49,51,114,51,116,24,234,29,
    184,17,91,208,158,32,203,12,39,30,96,19,172,179,216,110,
    100,176,109,211,230,49,176,237,75,73,120,117,104,132,134,180,
    61,71,172,114,199,108,65,137,154,171,3,223,135,3,112,54,
    142,128,243,30,201,52,209,3,231,24,131,178,136,159,9,163,
    110,246,54,39,61,156,167,250,64,73,136,180,142,65,228,199,
    212,51,143,154,227,125,1,99,207,8,191,204,128,145,228,54,
    178,186,174,97,167,59,75,42,102,97,56,139,41,200,179,96,
    22,179,10,131,179,138,207,56,171,224,204,132,243,57,125,24,
    152,124,30,232,78,142,108,181,97,194,76,47,91,136,11,216,
    118,162,112,167,91,10,55,74,138,141,65,177,251,254,181,120,
    229,90,124,15,163,114,233,1,199,67,29,151,117,228,141,100,
    135,34,39,77,125,188,83,151,124,44,243,55,199,209,129,210,
    225,160,233,244,142,123,68,228,52,89,218,72,182,128,143,140,
    88,69,116,82,12,118,19,138,233,38,144,78,95,145,20,69,
    222,1,83,204,34,250,138,130,69,117,244,209,193,233,33,63,
    197,207,151,180,43,100,14,9,148,252,219,85,173,8,235,72,
    218,218,159,30,66,216,160,52,180,43,184,228,239,18,100,229,
    15,144,69,31,51,241,162,191,2,103,212,2,254,2,132,29,
    132,72,207,139,82,167,35,176,76,209,240,223,3,187,219,49,
    25,10,199,180,42,101,37,60,2,67,93,124,135,135,234,132,
    229,43,248,91,198,87,147,180,194,236,229,204,217,180,194,74,
    227,33,131,238,76,169,131,117,56,112,210,174,109,186,49,13,
    211,209,240,192,253,15,206,162,52,187,197,211,96,96,8,28,
    214,235,59,36,234,247,7,248,163,131,121,78,76,25,25,84,
    125,78,205,205,20,80,34,161,13,66,234,69,56,57,189,112,
    244,57,245,29,137,102,177,50,227,67,234,19,252,143,12,159,
    84,31,58,143,158,254,250,233,90,213,209,188,147,111,125,107,
    164,174,150,75,92,237,102,234,106,146,79,220,87,92,152,81,
    107,16,132,246,13,129,21,53,166,168,84,192,90,32,115,80,
    203,147,83,114,165,33,122,62,43,146,104,74,177,247,208,113,
    206,214,92,211,118,78,81,164,1,66,205,206,96,163,20,97,
    228,126,203,109,175,123,238,131,144,100,32,65,234,137,23,27,
    137,86,19,89,173,200,3,197,73,138,241,215,213,68,187,237,
    193,70,168,219,184,100,170,21,251,163,23,214,57,44,125,179,
    41,75,109,217,94,199,138,125,211,239,148,54,90,110,131,247,
    210,236,105,253,52,209,90,49,24,250,83,170,120,137,218,176,
    84,15,3,60,108,182,234,42,140,74,158,196,202,85,122,165,
    229,18,159,84,37,63,46,185,235,248,212,173,43,237,81,135,
    163,5,103,251,110,212,136,57,177,127,241,146,186,131,199,130,
    227,248,129,143,181,207,31,33,205,24,116,33,157,30,60,92,
    213,104,7,197,195,29,107,82,213,213,1,149,210,43,123,133,
    154,27,240,94,156,79,183,112,73,90,54,38,3,231,197,156,
    81,48,212,130,14,22,199,206,249,154,56,198,71,227,195,45,
    113,134,248,160,239,218,122,81,34,79,35,229,16,93,177,80,
    91,160,19,171,86,76,136,35,220,142,50,113,44,33,158,227,
    118,156,137,19,9,113,146,219,243,76,156,74,136,23,184,157,
    102,226,76,66,156,229,246,34,19,47,37,196,203,220,206,49,
    113,62,33,94,225,118,129,137,139,9,177,196,237,85,38,126,
    144,16,63,228,246,35,38,94,75,136,31,115,123,157,137,229,
    228,46,242,6,19,151,160,246,9,93,196,17,229,83,138,144,
    67,239,26,33,57,144,12,62,132,236,252,95,3,163,125,231,
    253,82,202,190,11,189,92,237,164,160,40,178,26,143,233,160,
    216,20,73,41,154,85,151,47,208,174,157,234,99,78,61,146,
    174,146,122,143,231,7,109,14,14,192,90,162,63,29,132,189,
    163,133,210,195,84,243,125,206,101,187,23,120,235,117,173,206,
    91,47,158,5,151,177,98,178,184,98,186,79,21,211,46,155,
    201,49,116,209,116,0,246,92,106,45,186,212,8,228,203,19,
    5,212,22,211,53,18,9,234,118,58,50,240,236,37,200,150,
    61,252,120,176,152,162,240,255,35,100,178,76,83,92,192,58,
    231,168,255,211,185,151,177,4,99,32,151,122,252,192,209,192,
    206,241,247,196,57,202,203,144,61,252,236,251,212,240,113,151,
    158,116,246,207,211,189,188,119,6,228,163,10,152,17,200,200,
    143,21,38,180,114,71,57,235,93,44,54,168,84,127,167,249,
    152,45,243,237,199,73,3,212,205,55,101,174,101,122,139,105,
    36,202,248,17,81,98,245,211,211,89,97,244,113,48,169,82,
    206,134,31,181,49,39,146,78,219,109,134,17,75,242,14,211,
    73,162,89,156,126,194,243,183,100,237,7,239,36,25,79,127,
    141,100,244,156,171,199,83,88,31,88,232,204,131,105,85,242,
    57,173,253,89,166,165,186,158,121,112,186,6,235,81,57,125,
    90,36,91,210,141,165,227,97,32,230,165,222,116,14,173,72,
    55,76,89,98,82,30,190,150,73,216,118,98,255,7,189,232,
    155,140,167,5,41,107,78,8,234,238,233,147,49,90,69,202,
    15,26,142,235,121,100,164,70,91,226,174,211,202,111,61,153,
    196,160,16,117,236,83,117,227,116,182,219,120,162,244,182,247,
    236,163,105,85,58,144,244,87,181,116,150,137,250,253,40,173,
    243,6,195,105,161,33,72,95,175,242,73,195,73,134,135,251,
    140,219,254,122,54,138,162,117,239,166,219,195,48,24,133,93,
    199,209,119,111,248,189,229,56,239,65,165,242,51,92,242,37,
    173,77,201,0,86,42,34,143,181,202,180,56,230,215,40,228,
    11,130,139,200,190,23,204,90,1,154,175,239,157,186,177,77,
    20,123,60,61,156,248,205,103,82,177,209,57,198,247,170,107,
    110,91,191,154,226,183,43,246,135,208,187,213,182,175,167,135,
    28,189,8,224,203,62,125,17,139,201,10,23,184,92,207,218,
    95,16,253,35,114,243,213,149,68,227,149,199,59,234,209,225,
    99,104,91,82,141,204,47,97,219,171,156,243,100,199,203,96,
    171,237,28,153,164,174,246,13,59,158,231,226,113,204,250,56,
    45,245,141,233,219,146,94,203,149,252,86,36,213,220,177,195,
    171,126,91,191,113,84,147,125,207,189,200,197,254,116,31,53,
    198,213,221,22,133,5,50,125,66,230,18,244,148,245,79,187,
    210,234,27,207,149,110,18,62,206,122,15,198,30,17,201,6,
    26,8,17,113,168,46,238,99,223,203,64,9,164,103,137,142,
    89,190,131,247,45,125,195,165,223,22,60,160,183,89,241,175,
    176,161,87,147,133,241,2,250,25,229,169,166,40,98,166,106,
    153,163,19,5,107,116,164,96,21,134,76,126,55,52,38,166,
    140,162,85,24,25,21,135,127,23,209,7,139,198,226,229,130,
    248,31,150,55,240,207,
};

EmbeddedPython embedded_m5_internal_param_X86SMBiosBiosInformation(
    "m5/internal/param_X86SMBiosBiosInformation.py",
    "/mnt/hgfs/ShareShen/gem5-origin-stable-2015-9-3/build/x86/python/m5/internal/param_X86SMBiosBiosInformation.py",
    "m5.internal.param_X86SMBiosBiosInformation",
    data_m5_internal_param_X86SMBiosBiosInformation,
    2630,
    9150);

} // anonymous namespace
