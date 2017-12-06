#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_BasicLink[] = {
    120,156,197,89,109,111,227,198,17,158,37,41,217,146,229,243,
    187,125,119,118,99,245,229,26,245,80,91,201,37,206,165,136,
    235,54,41,82,160,65,225,164,84,138,187,40,65,89,154,92,
    73,148,41,82,32,215,231,83,96,127,169,15,109,255,64,127,
    64,63,244,67,255,77,255,81,51,51,75,82,178,236,11,14,
    104,33,219,210,122,57,220,157,157,151,103,102,103,215,30,100,
    63,37,252,254,186,14,144,118,4,128,143,31,1,33,192,64,
    64,91,128,144,2,252,117,56,45,65,242,62,248,37,120,5,
    208,54,64,26,112,133,29,19,190,54,32,170,241,156,50,132,
    38,83,4,140,170,32,45,104,151,224,89,180,2,150,44,195,
    105,21,146,63,131,16,34,18,240,220,159,3,127,30,94,33,
    119,236,84,152,225,60,16,177,202,196,10,248,11,76,172,130,
    95,227,206,2,140,150,65,214,160,189,72,195,218,247,144,237,
    99,100,187,196,108,255,67,108,125,124,179,1,254,61,26,142,
    114,125,69,35,45,26,201,235,45,49,151,229,92,202,21,104,
    175,230,253,181,137,254,58,247,113,165,85,232,111,64,127,19,
    250,91,128,6,241,87,10,174,247,65,154,208,127,0,237,7,
    32,241,115,31,174,208,62,254,234,196,140,135,60,99,173,152,
    177,205,51,118,160,189,3,18,63,219,122,70,25,90,141,77,
    180,121,240,95,252,105,160,205,65,213,176,121,33,147,52,136,
    35,39,136,58,113,96,208,251,50,53,228,33,143,154,185,204,
    85,191,33,87,253,27,216,79,190,145,185,234,18,144,177,32,
    93,66,3,46,185,115,105,192,168,1,23,2,250,22,248,38,
    92,224,50,37,18,160,43,224,202,128,111,76,26,112,137,173,
    133,6,125,11,44,165,253,212,103,131,106,78,115,112,89,130,
    139,18,180,158,95,24,68,56,173,64,242,47,248,118,135,153,
    206,51,83,3,46,176,181,224,202,130,203,50,60,195,65,72,
    234,87,72,125,241,252,2,53,69,74,171,97,161,180,199,19,
    234,146,42,126,144,68,238,64,170,101,236,59,67,55,113,7,
    206,39,110,26,120,191,15,162,211,70,53,31,21,167,251,67,
    87,245,108,158,102,146,61,6,67,197,236,226,72,170,5,236,
    116,130,200,119,6,177,127,22,74,53,79,188,156,78,16,74,
    199,225,151,191,27,12,227,68,125,154,36,113,98,147,73,153,
    24,198,110,49,131,12,234,133,113,42,27,180,26,47,99,19,
    123,69,163,59,67,230,72,2,176,168,52,217,151,169,151,4,
    67,133,158,210,28,105,52,113,107,144,143,184,73,29,108,154,
    131,72,53,123,221,78,218,108,245,220,68,182,122,50,106,118,
    229,224,96,47,78,130,110,16,237,165,202,61,9,229,222,147,
    119,222,61,216,251,197,222,123,205,147,179,32,244,155,47,63,
    252,160,57,28,169,94,28,53,7,7,205,32,82,18,141,20,
    54,167,204,179,143,67,86,105,161,243,160,235,4,172,162,211,
    147,225,80,38,139,68,125,72,66,136,101,81,19,101,97,138,
    134,88,196,94,9,191,166,216,49,22,196,113,64,74,122,164,
    56,225,203,154,68,20,185,89,192,169,1,201,14,225,165,143,
    31,65,14,70,212,180,232,157,193,239,254,64,214,209,212,190,
    73,40,208,196,11,198,24,130,13,71,30,146,219,35,96,160,
    148,160,95,6,13,32,196,157,70,84,50,162,22,135,19,27,
    3,153,91,144,254,3,208,218,8,157,11,200,96,117,101,130,
    136,150,65,85,41,154,145,186,137,11,254,133,145,217,106,144,
    248,199,140,16,213,11,210,248,60,98,63,80,159,99,169,133,
    150,249,98,244,249,73,95,122,42,221,69,194,87,241,89,221,
    115,163,40,86,117,215,247,235,174,82,73,112,114,166,100,90,
    87,113,253,81,218,32,215,218,43,57,200,10,126,163,97,14,
    42,2,0,130,74,63,248,129,167,240,97,141,31,216,11,169,
    84,8,144,94,236,167,72,39,22,93,169,108,18,82,145,145,
    99,22,132,241,227,208,80,90,30,199,221,195,231,143,115,73,
    24,164,141,114,14,169,84,134,29,85,101,116,186,105,234,176,
    36,68,103,32,18,227,23,110,120,38,153,59,66,73,161,64,
    212,213,50,204,24,138,247,73,173,220,10,172,90,20,71,254,
    8,37,13,188,183,73,136,251,12,200,26,67,114,3,225,56,
    135,109,25,255,150,197,166,225,89,25,8,203,57,16,41,53,
    42,96,24,136,12,9,8,202,43,76,67,13,131,243,8,107,
    199,145,250,35,234,209,100,123,135,154,31,80,243,22,53,187,
    185,1,102,103,133,197,105,43,60,165,149,13,86,157,149,36,
    167,153,185,146,254,181,104,123,48,142,54,76,156,45,138,26,
    131,98,107,28,53,22,37,217,228,136,90,28,202,241,104,66,
    250,37,165,116,138,46,102,70,129,132,33,65,189,113,160,176,
    201,108,74,178,141,249,28,227,54,1,119,18,189,221,9,244,
    218,228,45,134,174,253,32,207,152,14,141,208,160,181,183,137,
    85,233,22,155,215,169,249,225,236,13,63,134,95,247,6,252,
    62,34,33,150,51,248,45,50,236,170,248,93,54,60,51,243,
    70,177,163,174,77,193,142,48,103,221,130,185,159,82,207,188,
    169,255,157,193,45,211,250,183,19,112,35,65,141,73,229,142,
    177,51,218,34,157,38,129,182,133,133,194,179,104,11,247,126,
    131,247,254,119,120,239,231,250,129,43,39,157,208,77,206,233,
    186,83,34,227,116,76,216,204,246,244,180,130,237,48,137,95,
    142,234,113,167,174,88,123,202,191,135,143,210,253,71,233,71,
    152,89,235,71,156,211,116,110,213,217,51,145,67,202,126,52,
    245,211,151,158,228,189,148,159,28,71,39,59,135,19,159,147,
    237,209,136,185,13,50,173,145,219,156,211,126,170,18,202,246,
    51,182,122,181,176,58,41,241,25,45,91,101,147,155,98,11,
    241,85,21,44,155,163,243,61,87,109,252,22,191,159,144,27,
    72,127,9,84,87,219,45,45,57,43,69,234,217,63,191,134,
    161,153,169,100,55,113,141,63,230,216,41,143,177,67,95,51,
    15,140,191,1,87,182,2,254,10,132,14,4,65,22,24,69,
    28,17,28,214,104,248,159,128,35,232,150,58,130,243,82,139,
    106,7,30,129,233,42,125,202,67,117,89,241,25,252,125,34,
    252,242,205,223,204,106,215,201,205,223,42,114,26,195,234,141,
    54,120,235,122,242,35,55,245,220,148,134,233,140,54,142,232,
    241,6,82,20,157,152,209,103,135,177,121,189,160,67,178,125,
    51,70,24,109,159,219,98,205,152,192,205,187,212,60,41,32,
    35,114,218,76,196,220,133,215,239,250,142,222,77,190,38,89,
    44,150,126,105,142,35,188,224,80,196,70,41,143,141,39,69,
    108,72,222,245,94,241,1,135,90,131,32,112,101,8,60,93,
    98,33,72,135,57,11,100,9,218,101,138,34,46,224,69,22,
    100,34,207,119,148,29,175,109,169,108,156,99,109,182,2,5,
    218,193,212,188,156,113,30,33,31,31,134,238,224,196,119,143,
    250,180,40,173,236,229,97,103,228,106,44,79,170,65,33,35,
    94,167,9,63,30,228,234,188,152,113,14,249,0,88,72,173,
    6,71,140,31,123,156,56,190,236,201,250,64,14,78,240,108,
    219,11,134,245,78,232,118,217,91,102,166,230,231,185,154,138,
    221,61,93,184,164,143,169,141,235,94,28,97,194,63,243,84,
    156,212,125,137,71,62,233,215,247,234,188,91,212,131,180,238,
    158,224,91,215,83,58,4,174,199,51,87,205,110,210,77,185,
    64,62,61,167,238,29,120,219,193,115,125,128,135,134,16,138,
    109,90,31,57,139,228,207,199,1,29,81,184,163,226,97,78,
    141,116,142,163,34,198,222,167,230,103,112,55,123,196,251,160,
    47,34,82,50,97,89,108,27,21,67,45,77,134,243,23,52,
    39,189,25,212,255,124,147,160,214,151,69,89,104,151,105,164,
    156,163,251,5,106,43,180,77,180,171,57,113,129,219,26,19,
    23,115,226,61,110,151,152,184,156,19,87,184,93,101,226,90,
    126,115,181,206,196,13,104,111,210,149,15,81,182,40,135,204,
    253,175,57,132,35,239,14,98,46,253,191,166,14,251,233,29,
    107,97,127,8,89,189,241,186,180,33,38,85,92,212,105,163,
    47,242,35,209,164,126,124,55,179,121,19,163,142,151,72,87,
    73,237,182,157,153,43,204,73,72,139,240,98,156,9,110,22,
    236,31,23,186,93,113,197,53,90,103,111,234,83,33,123,83,
    60,139,30,98,229,110,113,229,126,72,149,251,5,27,194,49,
    116,241,62,6,108,169,176,7,153,54,146,231,206,148,77,116,
    113,78,146,185,195,161,140,124,251,49,76,214,219,252,122,198,
    184,160,156,119,9,19,197,143,41,214,177,192,190,25,165,148,
    221,39,116,101,183,150,138,184,156,189,131,25,209,175,114,68,
    55,248,48,93,164,120,251,144,26,78,234,69,62,183,127,85,
    184,231,39,183,192,245,196,141,252,243,192,87,61,167,227,210,
    246,71,199,189,55,27,136,101,25,95,174,78,191,80,219,183,
    204,14,17,144,145,55,98,238,223,247,158,152,82,186,204,158,
    111,31,139,93,39,240,95,207,43,123,95,240,210,207,234,225,
    45,99,207,101,208,237,41,102,245,61,175,137,19,193,87,63,
    50,120,56,246,125,25,74,37,167,225,174,175,156,245,78,235,
    75,172,28,226,17,30,57,249,224,134,207,161,227,220,197,22,
    251,75,92,35,161,197,214,129,183,88,81,198,77,118,67,240,
    175,81,41,87,4,87,52,83,255,23,208,50,210,21,132,62,
    166,140,82,155,243,232,82,1,41,190,191,206,171,9,66,31,
    151,225,199,238,64,223,55,242,13,154,253,99,200,238,53,236,
    183,11,104,210,221,15,159,13,245,201,28,179,6,87,91,92,
    92,217,239,17,157,122,131,131,253,92,169,125,173,84,43,24,
    232,107,87,190,65,31,28,168,149,169,97,126,226,98,127,99,
    138,154,202,36,112,195,224,91,125,221,154,147,185,200,152,230,
    75,242,20,79,92,154,92,207,243,236,216,68,118,131,20,153,
    48,135,98,116,150,238,200,214,106,231,22,56,77,206,188,3,
    16,232,99,129,190,245,56,162,123,183,244,8,27,186,53,173,
    44,85,16,16,148,5,77,81,197,60,104,153,181,229,138,85,
    91,168,88,149,57,147,47,181,22,241,104,88,181,42,11,53,
    65,191,187,8,152,170,177,187,86,17,223,1,132,108,67,237,
};

EmbeddedPython embedded_m5_internal_param_BasicLink(
    "m5/internal/param_BasicLink.py",
    "/mnt/hgfs/ShareShen/gem5-origin-stable-2015-9-3/build/x86/python/m5/internal/param_BasicLink.py",
    "m5.internal.param_BasicLink",
    data_m5_internal_param_BasicLink,
    2304,
    7142);

} // anonymous namespace
