#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_FaultModel[] = {
    120,156,197,88,253,114,219,198,17,223,3,64,74,164,40,139,
    178,190,108,75,182,216,118,220,176,158,74,140,157,40,78,39,
    170,219,164,19,207,36,51,85,82,48,29,59,76,166,8,68,
    28,73,80,32,192,1,78,150,152,145,250,71,229,105,251,2,
    125,132,254,209,183,233,27,181,187,123,4,8,81,114,38,77,
    51,148,76,156,15,135,187,189,253,248,237,199,93,27,198,127,
    5,124,126,91,3,72,254,44,0,60,252,9,8,0,6,2,
    90,2,132,20,224,173,194,81,1,226,119,193,43,192,107,128,
    150,1,210,128,11,236,152,240,149,1,97,133,215,20,33,48,
    121,68,192,168,12,210,130,86,1,94,132,203,96,201,34,28,
    149,33,254,6,132,16,161,128,151,222,28,120,243,240,26,169,
    99,167,196,4,231,129,6,203,60,88,2,111,129,7,203,224,
    85,184,179,0,163,42,200,10,180,22,105,90,235,22,146,125,
    132,100,151,152,236,191,137,172,135,95,214,192,187,69,211,145,
    175,47,105,166,69,51,121,191,37,166,82,77,185,92,134,214,
    237,180,191,146,235,175,230,250,107,185,254,58,247,145,131,219,
    208,223,128,254,29,232,223,133,14,42,101,57,219,237,30,72,
    19,250,155,208,218,4,137,191,123,112,129,122,243,110,231,86,
    108,241,138,149,108,197,125,94,241,0,90,15,64,226,239,190,
    94,81,132,102,125,29,109,225,255,7,255,234,104,11,80,21,
    108,94,201,56,241,163,208,241,195,78,228,27,244,189,72,13,
    89,174,77,205,220,216,132,191,35,19,254,11,216,126,158,49,
    54,225,57,32,97,65,178,4,6,156,115,231,220,128,81,29,
    206,4,244,45,240,76,56,195,109,10,196,64,87,192,133,1,
    95,155,52,225,28,91,11,21,253,0,44,165,237,215,103,69,
    107,74,115,112,94,128,179,2,52,95,158,25,52,112,84,130,
    248,159,240,237,22,19,157,103,162,6,156,97,107,193,133,5,
    231,69,120,129,147,112,168,95,34,241,197,203,51,148,20,71,
    154,117,11,185,61,200,137,75,162,120,126,28,186,3,169,150,
    177,239,12,221,216,29,56,207,221,227,64,253,62,242,100,80,
    47,167,211,162,100,119,232,170,158,205,235,76,82,200,96,168,
    152,94,20,74,181,128,157,142,31,122,206,32,242,142,3,169,
    230,137,152,211,241,3,233,56,252,241,147,193,48,138,213,199,
    113,28,197,54,233,148,7,131,200,205,86,144,70,219,65,148,
    200,58,237,198,219,216,68,94,209,236,206,144,41,18,3,204,
    43,45,246,100,210,142,253,161,66,83,105,138,52,155,168,213,
    201,72,220,36,223,96,211,24,132,170,209,235,118,146,70,179,
    231,198,178,217,147,97,163,43,7,123,59,81,236,119,253,112,
    39,81,238,97,32,119,158,188,253,120,111,231,87,59,239,52,
    14,143,253,192,107,156,190,255,94,99,56,82,189,40,108,12,
    246,26,126,168,36,106,41,104,76,235,103,23,231,220,166,157,
    78,252,174,227,179,140,78,79,6,67,25,47,210,232,61,226,
    66,84,69,69,20,133,41,234,98,17,123,5,124,76,177,101,
    44,136,3,159,164,108,147,228,132,48,43,143,41,50,180,128,
    35,3,226,45,66,76,31,127,130,76,140,184,105,210,55,131,
    191,253,129,212,163,71,251,38,225,64,15,158,49,202,16,110,
    56,115,159,12,31,2,67,165,0,253,34,104,8,33,242,52,
    166,226,17,181,56,157,200,24,72,220,130,228,31,128,234,70,
    240,156,193,24,88,23,38,136,176,10,170,76,126,142,163,235,
    184,225,95,24,155,205,58,177,127,192,16,81,61,63,137,78,
    66,54,4,245,217,155,154,168,153,207,71,159,29,246,101,91,
    37,219,56,240,101,116,92,107,187,97,24,169,154,235,121,53,
    87,169,216,63,60,86,50,169,169,168,246,48,169,147,109,237,
    229,20,101,25,189,209,48,69,21,33,0,81,165,95,60,191,
    173,240,101,133,95,216,10,137,84,136,144,94,228,37,56,78,
    36,186,82,217,196,164,34,37,71,204,8,3,200,161,169,180,
    61,206,187,133,239,31,166,156,48,74,235,197,20,83,137,12,
    58,170,204,240,116,147,196,97,78,104,156,145,72,132,95,185,
    193,177,100,234,136,37,133,12,81,87,243,48,107,44,222,33,
    185,82,53,176,108,97,20,122,35,100,213,111,191,69,92,220,
    97,68,86,24,147,107,136,199,57,108,139,248,127,81,172,27,
    109,107,140,194,98,138,68,138,142,10,24,7,98,12,5,68,
    229,5,70,162,186,193,161,132,197,99,95,253,41,245,104,177,
    189,69,205,125,106,30,80,179,157,106,96,134,106,88,156,86,
    195,83,218,218,96,217,89,74,50,155,153,74,233,93,242,183,
    187,19,127,195,224,217,36,191,49,200,187,38,126,99,81,160,
    141,159,81,139,83,217,35,77,72,190,160,176,78,254,197,196,
    200,149,208,41,168,55,113,21,214,153,93,37,93,204,167,40,
    183,9,186,121,252,118,115,248,181,201,92,12,94,251,110,26,
    52,29,154,161,97,107,111,18,169,194,53,74,175,81,243,147,
    27,208,252,4,128,221,43,0,252,128,184,168,142,1,184,200,
    192,43,227,83,53,218,230,216,28,89,90,93,153,2,30,161,
    206,186,6,117,63,167,158,121,85,1,55,7,184,177,216,207,
    115,128,35,78,141,188,116,7,216,25,109,144,80,121,168,109,
    96,185,240,34,220,192,10,192,224,10,224,109,174,0,184,138,
    224,186,74,7,117,147,227,186,238,20,72,59,29,19,214,199,
    153,61,41,97,59,140,163,211,81,45,234,212,20,139,79,49,
    120,255,97,178,251,48,249,0,163,107,237,25,199,53,29,95,
    117,4,141,229,144,34,32,45,253,248,180,45,57,161,242,155,
    227,232,128,231,112,240,115,198,137,26,81,183,70,186,53,82,
    165,115,232,79,84,76,17,127,214,106,47,103,106,39,41,62,
    165,125,203,172,115,83,108,32,194,202,130,153,115,116,208,231,
    226,141,191,226,243,17,217,129,20,32,129,202,110,187,169,89,
    103,169,72,62,251,151,151,80,52,59,153,236,6,110,242,199,
    20,61,197,9,122,232,49,83,223,248,27,112,133,43,224,175,
    64,248,64,24,140,125,35,115,37,2,196,10,77,255,19,176,
    19,93,83,77,112,108,106,82,5,193,51,48,100,37,79,121,
    170,46,46,62,133,191,231,60,48,45,1,204,113,13,155,47,
    1,172,44,174,49,176,190,87,154,183,46,7,64,178,83,207,
    77,104,154,142,106,19,167,158,100,145,172,246,196,168,62,67,
    148,205,235,29,29,98,238,235,9,198,40,137,110,138,21,35,
    135,156,199,212,60,201,64,35,210,177,217,240,185,13,111,78,
    254,142,206,41,95,17,51,22,179,191,52,199,238,51,33,145,
    249,71,33,245,143,39,153,127,72,78,126,175,249,172,67,173,
    65,40,184,48,4,30,64,177,34,164,243,158,5,178,0,173,
    34,121,18,151,242,98,236,104,34,13,122,20,34,47,101,86,
    86,207,129,86,92,6,4,109,99,106,78,103,29,76,200,204,
    251,129,59,56,244,220,103,1,237,74,91,183,83,215,51,82,
    57,170,121,57,200,109,196,155,68,225,215,189,84,158,87,179,
    14,36,239,225,38,153,28,236,54,94,212,230,232,241,69,79,
    214,6,114,112,136,7,221,158,63,172,117,2,183,203,246,50,
    199,114,126,150,202,169,216,224,211,21,76,242,136,218,168,214,
    142,66,140,251,199,109,21,197,53,79,226,241,79,122,181,157,
    26,39,141,154,159,212,220,67,252,234,182,149,118,131,203,78,
    205,5,180,27,119,19,174,149,143,78,168,123,19,246,118,240,
    148,239,227,1,34,132,44,93,235,243,103,150,3,248,104,160,
    189,10,51,43,30,236,212,72,71,58,170,102,236,93,106,126,
    1,55,148,42,222,37,221,209,110,164,196,162,216,52,74,134,
    170,94,242,233,207,105,85,114,213,179,79,190,143,103,235,75,
    165,177,127,23,105,166,156,163,251,6,106,75,148,46,90,229,
    116,112,129,219,10,15,46,166,247,86,183,120,112,137,239,130,
    138,60,178,76,225,97,238,255,13,15,236,83,55,225,77,199,
    63,106,84,176,159,222,180,24,246,251,48,174,39,222,20,17,
    68,94,198,69,29,17,250,34,61,246,228,5,228,43,152,141,
    107,192,231,180,99,233,42,169,45,183,53,123,145,57,194,104,
    30,78,39,94,126,181,40,255,48,147,238,130,107,170,209,42,
    27,84,159,253,216,160,226,69,120,15,171,115,139,171,243,125,
    170,206,207,88,21,142,161,11,244,9,104,11,153,70,86,177,
    9,229,137,51,173,21,93,129,19,107,238,112,40,67,207,126,
    4,249,162,154,63,207,26,27,20,209,206,33,87,223,152,98,
    21,171,232,171,190,74,193,59,39,45,91,182,144,121,231,13,
    216,152,97,253,58,133,117,157,110,228,38,17,220,222,167,134,
    99,118,22,174,237,223,100,22,122,231,58,204,30,186,137,12,
    48,161,57,29,250,226,188,146,148,228,28,207,69,62,241,11,
    29,242,126,208,58,172,203,20,149,179,223,61,77,61,190,142,
    180,146,3,204,62,174,58,142,165,115,34,253,110,79,37,151,
    25,250,223,87,17,59,156,141,191,99,18,163,130,29,27,137,
    74,37,175,32,153,51,206,248,238,194,147,152,242,163,17,30,
    25,249,220,133,239,129,227,220,72,106,252,181,246,102,29,152,
    49,53,138,34,38,199,53,177,102,148,138,37,193,117,200,212,
    213,190,102,144,74,104,125,194,24,37,54,135,200,165,12,40,
    124,3,157,150,0,132,41,62,37,31,184,3,125,97,200,55,
    96,246,207,96,124,43,97,191,149,1,142,174,110,248,88,167,
    143,213,24,14,184,70,226,146,200,38,24,241,173,201,96,111,
    55,149,104,247,121,16,185,41,44,248,2,124,176,167,214,167,
    230,124,18,102,51,54,167,62,105,133,52,253,129,190,118,229,
    171,253,252,119,47,118,177,191,54,53,154,200,216,119,3,255,
    91,125,207,154,14,43,82,192,52,65,146,35,123,227,50,100,
    42,244,51,28,98,217,245,19,164,194,36,178,233,227,248,199,
    6,186,127,29,100,243,75,111,2,59,250,32,160,111,59,158,
    209,141,91,242,17,54,116,97,90,90,42,33,142,40,46,154,
    162,140,145,209,50,43,213,146,85,89,40,89,165,57,147,111,
    179,22,241,60,88,182,74,11,21,145,254,219,70,188,149,141,
    237,229,146,248,47,213,233,92,103,
};

EmbeddedPython embedded_m5_internal_param_FaultModel(
    "m5/internal/param_FaultModel.py",
    "/mnt/hgfs/ShareShen/gem5-origin-stable-2015-9-3/build/x86/python/m5/internal/param_FaultModel.py",
    "m5.internal.param_FaultModel",
    data_m5_internal_param_FaultModel,
    2297,
    7168);

} // anonymous namespace
