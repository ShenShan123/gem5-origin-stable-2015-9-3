#include "sim/init.hh"

namespace {

const uint8_t data_m5_objects_Sequencer[] = {
    120,156,189,86,91,111,227,68,20,30,39,105,146,38,189,164,
    105,183,123,103,45,120,9,160,52,92,212,135,149,16,130,237,
    106,37,30,22,42,103,23,137,190,88,174,125,218,56,235,216,
    193,51,41,205,27,82,249,35,252,33,254,18,156,239,248,18,
    119,219,69,226,129,54,237,244,204,55,51,103,206,245,155,248,
    42,255,169,243,223,119,182,82,250,119,22,2,254,181,84,164,
    212,155,92,178,50,169,166,162,154,154,213,212,73,77,89,65,
    93,81,77,157,89,42,104,168,63,148,186,82,234,151,147,186,
    10,214,20,213,5,109,150,104,67,5,173,2,109,151,232,154,
    10,214,11,13,157,18,109,170,160,171,198,131,13,54,34,252,
    155,127,6,22,75,6,195,103,153,184,206,195,107,154,253,116,
    58,37,223,152,54,207,156,197,233,242,56,73,141,95,117,229,
    5,92,249,139,5,82,234,196,130,67,108,51,219,0,99,44,
    197,151,83,83,77,91,240,136,253,184,98,143,218,138,214,225,
    202,21,175,118,20,117,225,9,228,13,69,155,112,6,242,86,
    69,222,86,212,131,95,144,119,68,110,139,220,23,205,187,138,
    246,10,205,247,4,217,151,41,111,184,47,211,7,138,30,170,
    233,35,196,0,123,30,23,167,234,8,6,144,39,21,164,43,
    200,83,229,100,129,113,106,60,232,199,60,204,104,54,74,217,
    253,145,94,106,195,114,17,138,131,201,36,68,36,12,6,189,
    197,195,209,241,91,91,71,222,5,217,115,94,215,219,57,52,
    243,248,92,154,97,123,121,44,109,214,122,109,97,183,88,152,
    135,73,85,73,175,114,32,73,151,25,184,137,27,197,26,59,
    145,28,233,167,40,40,207,120,182,239,249,19,178,83,250,117,
    65,218,104,91,47,230,56,65,129,236,8,99,109,62,184,99,
    240,17,156,65,178,93,55,246,102,228,186,166,35,147,89,18,
    44,34,76,27,216,176,156,147,8,111,210,5,201,110,239,84,
    155,212,227,58,193,110,255,242,210,157,144,23,80,106,214,120,
    122,236,165,222,204,160,86,126,136,141,105,241,255,11,74,117,
    152,196,6,193,249,153,45,79,210,49,156,69,64,229,132,184,
    110,122,229,234,107,137,145,44,55,145,12,153,202,85,149,21,
    40,227,184,185,217,170,59,47,48,142,217,53,12,117,189,186,
    110,43,63,37,87,102,27,182,242,67,21,8,190,190,72,146,
    72,172,123,229,69,154,204,14,75,11,29,198,231,46,234,194,
    53,84,218,132,76,141,37,49,166,203,83,89,206,18,37,214,
    143,87,34,71,134,56,36,8,141,23,47,5,202,55,66,123,
    158,20,23,41,117,57,85,250,26,138,52,10,58,64,233,173,
    6,125,204,195,104,22,155,209,228,252,76,143,198,19,190,99,
    60,161,120,116,78,179,195,97,146,134,231,97,60,212,198,59,
    141,104,248,213,23,95,30,14,159,15,191,30,233,212,31,189,
    95,226,99,20,71,236,83,122,48,95,74,35,124,12,237,168,
    143,166,149,125,250,181,13,171,252,212,250,214,182,117,207,234,
    91,102,179,66,21,199,105,114,185,44,249,194,42,248,98,239,
    61,190,0,83,212,209,117,184,199,145,139,236,127,233,58,209,
    202,173,55,64,94,28,25,144,24,7,1,116,214,175,133,227,
    127,140,9,204,60,132,246,70,30,147,210,243,114,219,77,166,
    140,111,245,156,217,145,249,146,214,212,180,89,176,87,235,250,
    180,45,83,33,78,240,101,77,184,147,145,46,104,18,236,88,
    19,6,101,100,83,209,86,193,136,219,136,169,80,247,39,5,
    165,151,182,233,39,183,68,120,229,224,100,226,192,232,16,93,
    168,63,151,174,187,92,17,198,32,140,253,232,192,158,167,116,
    70,134,169,68,127,106,39,11,195,33,140,3,110,137,208,254,
    179,165,244,171,252,80,101,193,246,151,126,68,218,62,75,82,
    219,43,180,217,167,196,115,178,3,166,140,40,241,223,141,162,
    240,130,32,48,226,71,156,170,96,208,187,153,103,113,6,76,
    195,91,180,150,172,59,157,2,71,18,142,64,113,210,83,161,
    95,138,129,136,14,250,210,60,200,236,115,43,246,185,133,131,
    178,249,72,140,53,125,156,203,109,115,205,36,37,61,73,162,
    192,185,143,123,160,195,236,149,84,16,147,249,45,73,223,229,
    108,112,39,85,136,196,62,87,121,79,229,157,89,219,230,94,
    92,207,63,6,47,217,203,215,223,255,231,162,164,134,154,174,
    85,30,238,166,162,86,241,112,183,101,149,191,86,116,138,213,
    174,32,242,136,79,43,245,7,176,87,128,89,225,238,160,40,
    165,85,135,176,251,217,45,101,88,53,183,120,98,165,28,53,
    40,240,37,93,132,62,85,31,200,27,111,225,160,255,1,102,
    144,26,145,252,59,242,194,203,91,12,198,95,229,211,121,136,
    225,17,6,188,151,14,40,207,193,151,0,7,29,227,60,187,
    43,126,65,116,208,67,242,181,34,203,107,159,153,182,207,44,
    251,227,160,85,148,250,236,240,96,142,7,86,203,59,140,25,
    200,209,1,213,58,37,155,74,149,136,194,187,176,92,44,249,
    38,251,198,240,45,162,167,17,234,142,213,177,122,181,253,221,
    253,198,254,230,63,47,145,146,105,
};

EmbeddedPython embedded_m5_objects_Sequencer(
    "m5/objects/Sequencer.py",
    "/mnt/hgfs/ShareShen/gem5-origin-stable-2015-9-3/src/mem/ruby/system/Sequencer.py",
    "m5.objects.Sequencer",
    data_m5_objects_Sequencer,
    1145,
    2829);

} // anonymous namespace
