#include "sim/init.hh"

namespace {

const uint8_t data_m5_info[] = {
    120,156,173,88,91,115,219,198,21,166,147,54,201,224,7,244,
    249,76,94,34,79,120,145,19,219,177,21,143,39,32,9,73,
    72,121,43,0,90,81,94,50,32,176,36,17,1,88,100,23,
    144,194,252,192,254,166,206,244,165,223,89,128,23,145,114,51,
    211,169,199,166,73,236,158,219,119,238,136,90,205,159,103,248,
    247,3,181,90,250,111,248,18,183,90,63,227,243,89,235,231,
    103,173,248,147,214,207,159,180,226,79,91,254,217,95,112,164,
    255,245,69,171,53,75,69,168,5,105,33,40,201,227,228,62,
    137,171,48,165,101,146,10,77,75,169,40,22,101,152,164,154,
    228,146,202,181,160,52,137,68,142,251,50,39,17,70,107,115,
    177,107,5,56,41,148,88,10,165,68,188,187,19,133,57,45,
    4,184,84,121,12,230,52,114,7,206,196,119,186,150,101,167,
    91,17,120,92,174,19,77,113,162,75,149,44,170,50,1,231,
    51,9,81,10,7,96,96,46,8,18,191,151,184,163,68,84,
    74,181,121,78,235,240,94,88,141,28,77,11,88,16,179,74,
    124,179,239,15,9,122,143,221,96,171,136,238,18,249,50,19,
    135,34,13,71,107,199,145,66,37,232,106,50,167,209,213,108,
    132,235,19,73,181,14,59,25,124,97,103,9,171,108,29,170,
    12,155,250,98,35,113,186,85,161,33,108,147,62,148,28,165,
    85,44,204,157,165,76,83,249,144,228,43,138,210,80,37,203,
    36,10,153,145,117,132,115,168,73,137,223,170,132,113,93,108,
    204,81,36,139,141,74,86,235,146,214,50,141,133,186,176,44,
    194,159,224,128,106,33,192,155,45,20,121,12,215,229,41,40,
    229,1,97,3,129,150,203,242,129,13,11,243,216,240,208,235,
    16,158,201,101,201,126,139,100,14,3,43,8,134,18,43,21,
    230,37,107,27,238,132,128,99,152,111,106,156,12,117,146,151,
    34,77,1,40,71,80,161,100,33,84,185,105,140,102,82,64,
    101,120,167,73,150,148,96,11,6,135,36,134,199,142,76,137,
    52,52,2,89,12,252,173,98,163,105,146,21,169,200,68,94,
    26,180,154,168,52,148,203,42,143,248,89,152,38,32,111,96,
    220,89,216,40,29,19,148,21,240,162,80,240,242,173,172,40,
    11,55,84,105,177,99,179,163,208,213,226,87,40,198,242,15,
    29,82,10,149,233,6,96,40,139,124,97,75,214,97,73,27,
    89,25,14,184,86,41,81,63,51,177,13,155,65,76,9,187,
    178,72,217,209,32,169,242,76,198,112,59,195,91,7,85,82,
    234,134,190,132,187,13,114,196,238,56,140,51,125,108,88,155,
    118,108,100,237,133,61,227,54,115,208,178,82,17,59,19,113,
    135,180,192,147,69,146,135,136,120,36,119,134,176,13,158,136,
    168,125,164,158,177,191,34,132,125,237,138,147,68,68,236,13,
    118,196,103,209,115,250,230,252,252,188,243,205,249,139,23,38,
    30,61,177,130,53,70,105,254,57,207,147,123,112,111,252,51,
    78,162,117,178,10,243,35,14,47,222,190,61,111,227,227,219,
    14,62,94,181,193,240,59,102,120,110,56,92,139,135,84,148,
    101,103,22,70,119,8,9,26,138,123,120,162,224,128,160,129,
    204,10,4,228,41,187,183,160,63,127,219,54,90,141,67,117,
    71,195,46,93,39,64,150,129,31,134,112,33,217,93,186,145,
    50,62,53,230,109,109,140,237,141,105,84,135,237,233,157,55,
    134,63,205,20,80,19,37,162,114,111,231,233,229,239,80,153,
    102,62,5,34,90,231,50,149,171,68,232,54,185,121,212,253,
    184,236,248,62,4,227,152,1,83,146,77,70,52,125,156,232,
    9,164,29,164,223,162,82,171,245,19,234,176,238,111,12,205,
    101,42,85,18,135,228,35,183,196,127,179,1,190,24,135,90,
    163,252,35,113,74,184,215,69,161,72,202,10,84,236,232,173,
    101,79,120,226,156,125,250,237,113,104,148,39,10,15,144,196,
    136,207,60,9,79,53,126,93,163,61,9,77,123,232,39,249,
    29,234,197,233,181,58,0,39,162,228,98,124,201,133,219,84,
    140,143,192,246,162,9,218,171,16,133,175,159,34,186,78,149,
    127,9,79,132,25,93,165,176,253,35,166,125,83,59,151,35,
    17,104,40,242,55,186,20,217,71,125,245,146,60,228,0,199,
    241,143,136,191,112,149,163,144,158,166,211,158,157,147,175,146,
    92,8,101,234,48,98,23,7,89,149,55,157,131,109,125,144,
    234,78,211,40,92,60,133,135,163,146,136,126,132,105,90,62,
    149,115,255,91,110,189,100,179,95,195,97,42,23,136,229,206,
    24,229,252,81,6,28,155,189,205,236,151,52,88,43,84,54,
    89,112,155,189,234,66,92,38,80,231,80,68,79,8,76,144,
    2,219,240,78,162,26,143,195,82,87,153,52,237,39,166,32,
    188,171,126,77,104,146,232,117,146,85,234,56,96,64,253,166,
    109,204,31,135,121,37,82,234,203,106,131,226,111,233,127,254,
    181,213,122,124,247,29,66,68,188,167,119,199,229,240,189,153,
    87,204,19,46,225,90,168,123,17,163,116,122,226,209,212,194,
    218,112,39,217,23,93,126,114,80,109,17,7,15,73,185,230,
    50,204,255,203,170,180,234,82,29,53,161,201,109,7,221,15,
    101,134,59,196,227,238,114,52,51,96,212,72,234,134,0,34,
    11,200,93,64,179,147,78,113,88,252,179,74,151,184,131,105,
    174,110,254,225,66,222,31,148,126,171,238,82,237,186,101,165,
    224,196,12,14,229,52,195,205,94,9,136,195,236,146,100,66,
    125,111,29,11,127,220,103,182,194,97,82,92,69,226,255,46,
    191,25,104,172,88,70,213,126,56,0,69,15,88,215,131,92,
    6,223,170,36,196,36,187,195,213,56,131,89,30,170,254,189,
    149,139,164,25,63,5,229,97,38,182,37,234,180,75,230,114,
    127,203,224,205,45,28,42,215,204,36,110,240,116,129,138,82,
    233,122,220,193,56,134,167,166,17,67,11,196,48,143,205,12,
    9,2,11,28,145,51,49,45,113,96,25,16,118,163,72,19,
    45,164,11,17,113,184,128,40,225,32,82,28,40,121,29,50,
    90,215,99,104,112,237,250,228,79,47,131,27,219,115,8,223,
    103,222,244,131,59,116,134,212,191,165,224,218,161,193,116,118,
    235,185,87,215,1,93,79,71,67,199,243,201,158,12,241,116,
    18,120,110,127,30,76,61,223,250,210,246,65,249,165,57,176,
    39,183,228,252,52,243,28,223,167,169,71,238,120,54,114,193,
    12,220,61,123,18,184,142,143,226,54,25,140,230,67,119,114,
    213,38,48,160,201,52,176,70,46,230,111,92,11,166,109,35,
    244,148,140,166,151,52,118,188,193,53,126,218,125,119,228,6,
    183,70,222,165,27,76,88,214,229,212,179,108,154,217,94,224,
    14,230,35,219,163,217,220,155,77,125,135,216,172,161,235,15,
    70,182,59,118,208,201,221,9,36,146,243,193,153,4,228,95,
    219,163,209,99,43,173,233,205,196,241,88,245,67,19,169,239,
    96,29,177,251,35,135,5,25,35,135,174,231,12,2,182,102,
    255,109,0,224,160,222,168,109,249,51,103,224,226,11,176,112,
    96,139,237,221,182,27,158,190,243,143,57,46,225,144,134,246,
    216,190,130,105,103,127,130,8,92,50,152,123,206,152,85,6,
    12,254,188,239,7,110,48,15,28,186,154,78,135,6,103,223,
    241,62,96,93,242,191,167,209,212,55,96,205,125,167,109,13,
    237,192,54,130,193,2,72,225,24,223,251,115,223,53,152,185,
    147,192,241,188,249,44,112,167,147,231,112,239,13,80,129,142,
    54,72,135,6,220,233,132,77,69,140,56,83,239,150,153,50,
    6,6,251,54,221,92,59,120,238,49,158,6,41,155,33,240,
    129,216,32,56,188,6,121,0,48,176,246,54,210,196,185,26,
    185,87,206,100,224,240,233,148,185,220,184,190,243,28,174,114,
    125,190,224,26,177,112,62,100,206,141,201,236,34,104,101,153,
    175,7,1,219,54,142,36,247,146,236,225,7,151,213,110,46,
    195,245,190,219,132,137,129,108,112,221,192,141,82,254,239,207,
    90,173,128,51,6,127,57,31,87,34,123,69,26,237,0,43,
    132,84,205,144,155,113,225,123,16,11,52,165,163,213,20,213,
    117,93,150,197,69,175,247,240,240,208,101,226,174,84,43,108,
    169,180,194,76,72,186,12,149,25,127,11,137,117,133,101,60,
    113,187,231,34,235,77,38,215,149,28,108,81,248,80,223,121,
    103,201,185,6,54,53,105,193,137,188,168,146,52,222,78,212,
    59,69,13,213,10,45,152,79,140,84,76,155,197,126,63,127,
    74,236,240,184,226,89,79,221,10,184,26,113,241,99,44,100,
    45,222,160,212,230,173,5,245,133,183,62,33,142,11,236,182,
    0,93,208,234,235,175,185,104,161,220,230,171,182,53,219,160,
    30,97,77,55,56,167,152,192,118,75,117,115,194,107,157,42,
    208,108,132,122,142,32,26,160,126,227,191,27,23,249,240,71,
    154,44,176,180,188,108,91,108,45,102,169,18,203,41,138,96,
    41,23,21,10,232,178,86,8,203,38,149,42,228,222,21,22,
    101,85,175,168,84,164,225,102,193,179,153,174,138,66,170,178,
    75,179,63,65,71,20,40,185,34,143,146,230,109,134,241,71,
    243,74,131,43,117,36,48,100,54,126,200,146,28,158,200,200,
    12,46,7,171,86,8,66,179,114,226,161,41,227,210,224,56,
    5,177,81,150,223,69,212,139,218,161,52,204,9,50,189,231,
    37,172,220,20,130,190,210,188,78,91,6,249,222,59,27,117,
    239,125,175,86,179,40,191,162,7,222,74,137,159,114,120,65,
    12,203,182,71,179,107,36,59,86,143,54,77,230,35,212,30,
    30,46,129,62,242,218,3,156,168,141,131,54,123,229,167,55,
    175,187,100,226,223,56,178,246,46,2,28,172,177,178,252,1,
    157,27,147,182,22,177,96,171,105,207,103,91,45,158,27,128,
    248,216,4,101,221,106,120,59,85,88,211,74,94,209,149,232,
    146,47,196,147,17,214,103,161,191,212,243,238,9,210,198,121,
    172,13,32,0,114,55,219,190,187,143,124,214,185,108,55,80,
    82,42,229,29,178,242,73,65,94,149,179,199,126,49,177,183,
    19,116,152,98,248,187,198,114,142,118,139,182,107,25,194,186,
    2,44,66,141,222,217,204,68,74,212,161,211,108,186,166,112,
    104,179,245,111,183,91,120,241,130,55,234,14,79,33,203,100,
    165,47,176,252,134,252,254,97,171,56,75,171,207,48,120,154,
    95,58,82,73,81,239,241,29,94,149,47,40,21,90,119,34,
    12,234,252,198,236,119,36,68,206,47,71,120,196,94,65,40,
    39,93,61,26,236,115,178,38,214,42,186,120,188,188,47,159,
    168,109,205,93,3,251,238,58,163,98,222,58,213,144,67,94,
    125,97,63,80,152,27,53,15,72,175,79,27,165,75,161,75,
    205,195,228,10,17,108,130,198,60,169,15,49,33,165,23,12,
    235,178,74,205,15,222,215,144,188,43,21,102,181,151,205,139,
    46,83,100,84,149,19,174,165,157,173,240,29,102,250,184,234,
    0,158,2,116,91,77,192,68,101,172,167,117,198,73,35,211,
    58,251,103,246,200,0,193,202,219,105,177,14,81,90,176,251,
    229,24,234,77,44,115,214,153,64,203,205,128,85,7,96,162,
    239,172,36,99,172,31,21,139,29,146,177,124,200,83,25,162,
    182,224,206,54,3,56,44,24,145,143,197,224,176,33,178,44,
    119,185,47,1,191,85,0,170,54,111,87,180,161,14,250,78,
    202,14,102,234,14,160,83,250,135,125,139,113,242,95,37,191,
    127,226,26,100,244,169,203,92,77,189,102,87,129,189,162,195,
    53,161,54,210,188,219,211,117,66,77,206,62,109,181,90,229,
    231,248,224,177,7,29,215,124,111,94,183,150,104,142,45,207,
    177,135,99,231,140,223,14,239,63,244,223,241,209,203,242,178,
    183,94,45,117,207,103,121,254,90,228,166,48,117,144,0,216,
    54,59,232,67,139,84,240,118,252,170,243,182,243,109,175,174,
    96,191,191,121,221,43,76,173,239,101,175,122,156,125,93,252,
    252,2,252,222,65,215,42,21,239,249,37,180,230,183,204,159,
    61,251,236,217,127,0,92,110,123,143,
};

EmbeddedPython embedded_m5_info(
    "m5/info.py",
    "/mnt/hgfs/ShareShen/gem5-origin-stable-2015-9-3/build/x86/python/m5/info.py",
    "m5.info",
    data_m5_info,
    2682,
    5787);

} // anonymous namespace
