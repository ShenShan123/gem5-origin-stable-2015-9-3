#include "sim/init.hh"

namespace {

const uint8_t data_m5_objects_SouthBridge[] = {
    120,156,181,87,91,79,27,71,20,62,235,27,182,177,49,119,
    72,200,101,201,213,73,11,6,3,185,72,85,21,64,170,68,
    149,4,100,231,161,241,203,118,217,25,236,133,189,88,187,235,
    8,170,84,170,68,255,69,95,250,83,250,47,250,55,250,208,
    231,182,231,156,217,49,118,146,86,125,137,7,31,206,249,230,
    204,217,153,217,239,156,25,59,144,126,178,248,125,97,2,196,
    191,163,34,240,207,0,15,224,77,170,25,74,203,128,151,1,
    63,3,157,12,24,100,103,193,203,130,159,133,78,86,217,57,
    240,114,224,231,160,147,83,118,30,188,60,248,121,232,228,149,
    93,0,175,0,126,1,58,5,101,79,128,55,1,254,4,116,
    38,148,93,4,175,8,126,9,58,37,101,151,192,43,131,95,
    134,78,89,217,101,240,38,193,175,64,167,162,236,73,240,170,
    224,79,65,103,10,237,10,252,12,208,169,129,168,130,156,130,
    19,3,196,20,33,151,0,111,59,211,32,106,208,174,79,227,
    218,220,191,241,83,55,80,75,72,60,86,106,14,197,190,31,
    198,202,202,163,56,120,182,177,221,84,102,145,205,230,198,243,
    237,221,221,81,135,230,214,211,49,115,103,123,220,124,174,204,
    42,153,66,238,135,65,18,133,158,39,35,5,151,80,28,57,
    237,190,180,207,52,84,70,241,221,179,39,7,65,242,210,13,
    228,149,91,219,245,15,143,79,165,147,56,4,101,210,239,62,
    189,174,41,224,55,244,163,1,239,241,15,96,169,93,167,190,
    215,94,30,174,62,69,168,211,63,238,225,165,246,195,40,73,
    22,104,94,135,150,45,68,36,227,216,138,251,182,35,173,99,
    59,150,202,153,68,252,45,138,134,31,36,141,94,247,36,110,
    180,123,118,36,219,61,25,52,186,210,223,89,11,35,183,235,
    6,107,113,98,31,123,114,173,185,177,185,179,246,124,109,171,
    17,71,78,67,200,119,141,243,103,79,26,237,112,144,244,246,
    34,87,116,229,122,255,34,169,96,56,132,15,14,119,213,67,
    31,211,51,104,66,96,20,140,100,146,150,122,53,192,209,211,
    167,45,216,163,197,110,225,2,36,190,100,131,24,138,36,196,
    133,35,245,100,14,78,243,32,11,112,58,65,20,189,68,126,
    22,65,150,136,158,178,76,164,188,52,136,138,200,190,75,200,
    116,38,71,186,38,116,87,145,187,42,32,171,186,171,68,93,
    151,96,32,181,100,77,131,101,13,78,131,156,33,250,17,88,
    225,32,85,214,167,148,67,166,51,11,114,78,143,170,233,81,
    243,32,23,52,56,173,193,69,144,75,4,138,25,54,151,121,
    57,37,144,147,32,102,121,45,215,52,82,1,49,199,200,117,
    70,152,230,98,158,145,21,70,106,32,145,232,11,140,220,96,
    100,6,228,44,136,69,70,110,50,50,7,114,30,196,18,35,
    183,24,89,0,185,8,98,153,145,219,140,44,129,92,6,113,
    141,17,19,228,42,136,235,208,69,154,173,208,14,136,27,32,
    110,130,184,197,187,1,185,206,29,16,56,234,14,88,119,65,
    222,99,229,62,8,147,149,7,32,86,89,121,168,187,234,32,
    238,176,242,72,35,143,65,220,101,229,11,237,252,165,238,90,
    3,161,148,117,122,22,41,13,16,247,89,217,0,241,128,149,
    77,237,211,188,122,65,100,110,129,120,200,245,96,27,90,237,
    58,113,185,69,153,18,47,81,186,164,220,140,137,106,214,177,
    34,103,175,23,223,166,132,244,236,228,36,140,124,51,233,185,
    177,137,158,174,35,77,212,250,118,148,152,225,9,151,130,190,
    27,114,210,184,166,78,39,63,20,146,217,203,89,255,202,142,
    19,25,185,191,234,140,102,176,237,217,239,164,219,167,2,4,
    58,173,133,157,216,22,165,162,251,61,65,148,28,78,232,251,
    118,32,20,42,200,251,5,9,155,194,0,252,246,23,39,37,
    5,85,15,49,143,14,246,99,174,16,20,158,173,7,84,25,
    94,29,182,77,95,250,97,116,97,98,52,51,146,182,103,38,
    174,47,77,199,11,157,179,116,93,241,50,186,190,233,73,243,
    196,141,226,196,20,190,109,58,195,26,197,59,69,157,103,242,
    226,56,180,35,49,218,183,66,59,21,133,221,200,198,217,98,
    238,155,110,128,147,121,151,62,36,226,9,30,237,155,177,170,
    109,49,215,207,198,161,185,139,243,227,226,40,220,248,44,86,
    123,233,184,214,201,32,112,146,137,212,160,169,209,150,14,129,
    227,65,236,254,129,101,207,253,19,143,168,132,42,216,214,158,
    219,39,32,65,192,165,130,232,254,68,130,222,175,147,77,143,
    178,138,174,143,69,28,145,112,137,196,226,120,106,192,105,134,
    106,4,233,89,56,205,81,226,169,140,189,242,201,143,248,24,
    236,147,251,200,71,215,154,212,255,147,113,138,35,62,198,255,
    240,201,254,203,179,74,112,90,30,247,201,127,202,103,114,220,
    167,144,250,116,39,8,180,42,233,84,213,100,172,106,250,116,
    146,83,96,213,82,92,13,183,166,233,44,193,218,161,86,106,
    205,142,152,115,227,230,252,71,230,2,188,207,192,233,98,106,
    98,41,161,226,152,133,179,12,68,191,24,228,89,213,158,75,
    16,192,200,216,210,120,40,99,220,204,143,155,197,113,115,106,
    220,204,106,179,170,205,101,58,48,218,117,226,212,235,164,64,
    57,20,14,34,71,114,242,198,110,112,198,57,233,18,185,20,
    161,136,62,245,107,84,53,74,195,35,211,117,54,121,40,86,
    141,254,32,97,126,82,33,64,156,136,136,253,129,246,107,178,
    226,224,117,66,121,5,137,69,189,202,43,97,222,235,172,74,
    166,71,12,75,123,86,185,166,12,98,57,68,74,105,28,15,
    239,5,49,231,80,76,25,207,241,211,52,99,212,165,91,8,
    7,149,231,152,146,129,237,165,33,28,158,187,207,85,35,157,
    73,200,211,196,180,223,100,192,197,26,86,224,18,20,156,184,
    93,142,230,132,131,32,225,78,244,226,90,70,193,84,144,122,
    86,239,76,44,189,19,118,194,84,85,117,205,87,101,45,254,
    124,23,9,218,67,59,73,108,167,119,112,248,150,226,55,233,
    41,217,123,134,110,179,220,86,50,53,3,155,146,216,86,16,
    171,141,180,250,14,164,247,59,203,10,108,95,90,22,175,210,
    178,176,162,15,60,50,185,22,93,244,37,227,206,249,185,213,
    147,182,72,55,251,200,198,2,200,163,245,177,193,27,136,176,
    76,183,205,14,212,84,251,105,127,139,250,91,244,122,57,128,
    197,164,210,90,179,53,188,61,90,68,158,86,110,104,210,75,
    106,101,244,185,97,105,194,180,242,250,29,224,248,164,85,28,
    46,38,165,68,43,59,68,82,170,182,110,208,4,232,224,106,
    173,146,88,39,113,143,196,93,18,143,72,220,34,65,196,106,
    53,116,248,189,221,214,6,43,111,162,129,228,131,138,144,151,
    178,107,59,23,7,135,218,101,147,31,69,74,219,253,97,232,
    182,249,161,91,83,247,52,63,236,217,210,1,182,70,3,108,
    125,232,182,205,180,223,87,71,165,218,114,60,140,14,190,81,
    55,109,58,138,162,65,159,175,208,28,98,136,28,33,123,9,
    208,241,246,240,174,219,122,170,41,250,217,200,202,55,15,218,
    232,248,37,138,130,65,109,33,179,130,100,92,192,70,132,164,
    255,181,204,172,49,214,178,171,70,233,63,91,45,243,186,62,
    163,57,225,239,172,247,137,142,234,88,37,43,10,207,47,152,
    81,76,28,166,2,51,138,73,195,68,100,138,226,47,19,245,
    178,139,58,148,250,5,130,155,165,138,95,69,197,27,254,6,
    105,149,53,135,121,97,159,119,239,120,53,95,169,108,252,250,
    38,197,191,142,162,108,148,141,233,143,91,166,148,251,7,144,
    31,237,72,
};

EmbeddedPython embedded_m5_objects_SouthBridge(
    "m5/objects/SouthBridge.py",
    "/mnt/hgfs/ShareShen/gem5-origin-stable-2015-9-3/src/dev/x86/SouthBridge.py",
    "m5.objects.SouthBridge",
    data_m5_objects_SouthBridge,
    1699,
    3792);

} // anonymous namespace
