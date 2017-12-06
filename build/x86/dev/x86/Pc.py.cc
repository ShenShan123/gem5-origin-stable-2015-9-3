#include "sim/init.hh"

namespace {

const uint8_t data_m5_objects_Pc[] = {
    120,156,181,85,77,111,219,70,16,29,146,18,37,74,178,229,
    196,113,156,84,45,208,91,133,22,178,98,39,46,82,160,40,
    234,164,40,224,139,109,80,233,161,188,44,86,228,74,90,135,
    31,2,185,14,164,162,5,10,56,255,161,183,254,190,92,122,
    233,169,183,118,223,82,140,197,246,154,138,210,104,222,12,185,
    154,125,243,102,21,210,230,229,232,207,183,159,18,21,191,105,
    39,210,111,139,98,162,87,27,207,42,61,155,98,155,18,135,
    2,135,44,96,135,226,6,37,77,10,154,37,110,80,236,82,
    226,82,224,150,184,73,113,139,146,22,5,173,18,187,20,183,
    41,105,83,208,46,113,139,98,143,146,14,5,29,141,219,244,
    150,40,232,82,228,145,112,105,102,81,212,65,228,150,232,199,
    160,71,81,151,38,195,158,174,77,254,173,95,67,75,123,10,
    230,243,210,109,105,115,94,240,239,249,107,81,6,112,235,85,
    40,95,102,233,76,206,207,226,184,140,182,17,141,185,154,101,
    121,82,70,186,218,76,178,27,181,120,145,203,104,46,238,110,
    123,37,242,68,166,124,235,193,31,120,174,158,159,156,62,9,
    17,177,55,159,151,224,108,151,12,77,191,88,244,179,126,19,
    29,78,134,200,93,196,77,186,123,181,105,136,47,147,81,13,
    109,150,89,174,212,1,10,191,100,60,138,114,81,20,172,88,
    242,80,176,41,47,68,121,51,76,113,166,205,56,73,213,120,
    49,159,21,227,201,130,231,98,178,16,233,120,46,146,211,81,
    150,203,185,76,71,133,226,211,88,140,78,158,28,159,142,190,
    26,61,29,23,121,56,142,196,155,241,234,249,151,227,171,240,
    104,185,54,140,104,116,126,121,86,254,214,103,88,26,117,144,
    229,90,202,54,124,133,85,177,40,252,5,182,134,237,11,221,
    23,11,162,8,108,108,83,119,95,52,232,186,137,54,93,183,
    160,138,91,27,45,21,30,154,165,123,41,186,198,233,145,216,
    129,70,196,46,148,113,107,65,15,90,2,183,100,7,253,173,
    84,171,158,218,219,74,181,171,148,103,82,247,72,220,55,43,
    239,155,108,7,89,177,79,236,1,137,3,196,225,63,220,122,
    188,91,127,252,112,43,213,171,167,30,109,165,118,234,169,199,
    91,169,221,42,213,55,169,143,104,174,251,190,71,111,53,39,
    3,242,39,195,123,154,53,191,3,94,119,160,136,13,253,203,
    240,104,177,80,46,194,235,66,137,196,136,105,41,51,211,116,
    249,107,37,47,68,10,249,147,144,144,151,252,3,230,47,221,
    51,217,54,158,3,163,155,36,223,193,123,7,239,79,120,232,
    91,232,108,6,216,169,4,249,187,118,180,14,175,245,187,212,
    164,233,145,165,189,107,219,36,28,98,13,186,131,205,58,116,
    235,176,85,135,237,58,244,234,176,83,135,221,10,246,12,220,
    1,212,74,154,152,177,186,24,238,86,179,90,96,6,217,212,
    12,161,161,131,43,197,195,197,249,165,33,46,225,154,184,92,
    129,85,201,162,44,85,76,172,100,161,142,149,83,18,247,159,
    204,137,66,27,166,98,33,211,136,45,67,169,32,232,48,75,
    216,177,73,204,244,57,193,0,79,234,240,105,29,62,51,199,
    131,129,179,56,91,234,33,106,153,190,206,248,77,172,148,135,
    223,14,101,104,142,152,161,83,141,117,33,226,153,169,107,122,
    83,152,123,162,132,51,204,122,241,161,71,218,239,227,240,192,
    114,15,204,24,239,91,253,127,93,195,143,43,117,49,150,242,
    68,48,102,54,200,88,146,69,55,49,32,42,86,235,165,48,
    241,112,181,98,11,193,35,77,53,248,186,226,57,79,12,255,
    147,82,184,110,25,20,169,50,27,228,233,218,7,65,62,68,
    232,15,96,176,156,143,166,250,232,175,143,237,251,102,42,246,
    97,80,166,143,69,124,28,123,62,250,238,55,171,10,213,230,
    192,245,31,34,124,8,243,8,230,49,76,191,98,238,131,115,
    136,125,127,129,229,244,220,144,107,225,58,176,61,219,115,6,
    214,160,49,112,60,205,98,15,190,53,112,6,246,197,16,66,
    51,93,77,78,143,150,224,167,48,197,3,229,217,106,109,40,
    250,78,188,145,161,48,12,24,158,244,255,80,73,145,243,158,
    162,102,165,22,252,169,148,148,120,213,225,241,191,236,211,84,
    249,117,217,246,111,62,193,178,247,181,233,88,29,123,207,122,
    127,217,94,227,31,9,116,137,181,
};

EmbeddedPython embedded_m5_objects_Pc(
    "m5/objects/Pc.py",
    "/mnt/hgfs/ShareShen/gem5-origin-stable-2015-9-3/src/dev/x86/Pc.py",
    "m5.objects.Pc",
    data_m5_objects_Pc,
    969,
    2065);

} // anonymous namespace
