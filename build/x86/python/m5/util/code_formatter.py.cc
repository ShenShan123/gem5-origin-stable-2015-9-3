#include "sim/init.hh"

namespace {

const uint8_t data_m5_util_code_formatter[] = {
    120,156,205,90,205,111,28,71,118,175,234,158,25,114,134,67,
    81,20,69,234,91,110,219,98,52,116,76,81,246,174,189,182,
    87,43,219,178,100,197,89,155,246,54,101,41,75,153,233,109,
    78,55,201,38,155,221,227,238,30,137,50,57,200,198,92,196,
    82,176,217,5,22,65,190,78,123,72,128,4,1,2,4,48,
    16,32,57,228,15,200,53,155,91,254,134,220,115,72,242,126,
    175,170,122,134,164,108,36,7,207,134,51,172,121,93,213,253,
    170,222,71,189,175,234,182,208,127,85,250,127,203,17,34,255,
    103,41,68,64,95,41,98,33,150,75,88,138,101,105,96,75,
    44,91,6,182,197,178,109,224,138,88,174,136,192,18,97,85,
    172,209,205,182,248,137,16,251,66,252,112,185,38,130,138,8,
    71,184,183,90,246,142,138,160,102,238,29,41,123,235,232,93,
    167,233,26,34,28,19,193,168,216,178,68,246,79,18,83,212,
    69,92,23,219,117,65,247,200,176,142,219,151,155,34,160,251,
    198,25,199,88,137,227,152,8,154,98,121,66,132,4,140,115,
    75,223,9,26,147,132,88,181,152,225,184,208,88,38,69,56,
    41,130,227,98,95,18,98,66,57,89,66,39,74,104,170,132,
    78,150,208,180,129,54,79,0,143,220,241,68,56,37,130,25,
    116,223,92,249,129,88,62,41,14,220,22,156,42,161,211,34,
    56,35,246,45,117,113,22,23,193,57,177,111,171,235,243,34,
    184,80,14,94,52,23,159,222,146,247,48,215,52,207,133,161,
    103,68,224,48,97,234,185,219,9,203,97,169,245,44,9,51,
    250,111,250,91,44,106,4,198,105,186,213,237,180,141,168,73,
    192,226,6,68,253,60,1,161,208,130,37,222,65,170,146,1,
    27,98,4,80,17,238,82,203,166,251,218,182,86,18,139,254,
    63,196,211,111,17,176,103,137,61,33,60,2,36,3,220,110,
    74,177,169,250,109,177,103,51,80,17,123,21,6,170,34,18,
    12,212,48,231,82,11,107,89,108,141,80,91,0,249,90,230,
    111,135,69,29,80,154,109,251,69,17,102,220,239,5,81,187,
    40,42,4,149,128,159,173,231,76,221,214,195,18,140,211,182,
    31,231,45,60,193,247,228,97,188,230,98,193,46,230,113,209,
    239,226,190,150,48,77,126,135,154,133,237,164,88,216,88,95,
    203,23,150,54,252,44,92,218,8,147,133,245,112,251,149,249,
    52,139,214,163,100,62,47,252,213,56,156,127,249,234,75,175,
    204,191,62,255,173,133,60,107,47,116,30,21,27,105,178,176,
    253,202,66,183,136,226,133,118,26,132,94,185,230,43,52,58,
    138,101,123,81,18,21,158,247,28,102,106,50,239,235,244,153,
    224,150,57,106,254,223,1,71,39,75,142,110,50,71,175,13,
    176,136,9,0,159,88,26,238,168,33,49,42,194,237,2,93,
    15,252,120,24,100,141,49,89,121,88,96,98,207,123,1,147,
    89,76,89,219,210,218,81,53,244,44,72,163,24,68,207,86,
    77,100,103,251,180,157,89,218,51,202,130,145,111,107,216,140,
    144,30,98,223,127,192,221,150,216,180,197,102,69,240,128,197,
    3,109,51,80,165,94,130,106,64,155,253,220,76,71,3,35,
    140,247,231,253,203,193,73,71,121,240,111,52,172,70,20,150,
    186,200,126,53,128,165,193,55,254,170,127,105,176,16,35,54,
    155,60,248,31,26,230,145,71,231,68,49,14,144,246,121,79,
    97,57,166,70,238,37,147,162,82,76,136,173,134,200,46,72,
    250,75,164,248,157,226,56,204,36,141,207,124,110,169,13,1,
    153,46,106,229,89,139,226,208,243,244,69,28,37,116,145,79,
    129,185,105,55,14,156,36,45,156,181,40,9,156,203,179,249,
    229,214,164,209,15,86,112,214,119,222,18,107,30,100,200,98,
    107,167,140,49,193,22,27,229,33,224,76,82,222,33,166,135,
    119,144,139,85,20,192,182,30,167,171,212,161,182,164,103,174,
    148,18,172,118,163,184,136,146,114,133,216,154,116,1,101,140,
    146,130,119,91,209,160,230,174,31,119,195,91,89,150,102,124,
    249,94,18,132,59,124,217,178,140,42,187,141,33,237,74,181,
    242,117,163,190,243,152,236,69,86,223,9,57,102,225,191,41,
    199,233,191,97,93,148,199,248,218,252,218,178,73,247,140,203,
    138,84,246,69,209,12,102,18,205,13,190,216,78,131,46,36,
    230,130,91,46,246,187,123,246,0,93,223,44,113,46,176,195,
    244,231,16,124,77,214,71,235,118,49,197,130,31,188,211,35,
    241,251,71,157,193,169,65,103,48,232,9,200,1,88,108,75,
    169,109,56,244,215,122,243,13,254,117,156,217,86,16,198,209,
    246,92,222,122,243,163,107,97,222,246,59,97,112,189,236,156,
    115,14,252,237,57,207,59,250,30,135,239,32,17,100,26,209,
    239,2,1,233,114,152,20,215,239,59,43,47,204,29,192,28,
    167,73,72,104,35,12,19,218,75,140,10,157,14,119,69,107,
    81,137,232,192,115,60,58,240,160,227,28,93,209,87,99,152,
    109,197,171,10,207,170,119,24,211,108,43,163,49,133,97,53,
    243,219,68,210,215,47,165,147,230,244,56,181,79,89,134,89,
    10,141,70,69,148,38,126,236,116,124,118,132,95,191,168,65,
    156,102,65,6,151,94,84,31,229,83,16,5,26,83,72,174,
    227,250,149,23,222,4,146,160,196,162,17,5,105,151,20,209,
    224,11,119,58,89,152,231,132,241,169,252,78,8,83,20,92,
    127,42,133,244,71,220,142,227,121,40,226,160,10,48,210,156,
    241,205,113,219,134,245,25,27,116,36,255,133,13,71,95,54,
    168,20,10,33,188,144,236,42,109,142,125,72,87,9,234,251,
    17,91,244,42,34,249,37,195,21,241,137,37,122,85,209,171,
    241,101,85,124,82,17,189,17,209,27,21,189,186,232,53,128,
    148,238,143,70,96,46,225,37,216,35,236,179,163,121,21,23,
    117,88,104,134,26,136,98,95,213,183,85,249,158,106,121,93,
    227,235,90,121,77,223,81,113,106,159,67,91,211,87,23,123,
    13,213,55,42,94,157,161,37,162,127,12,161,17,128,38,183,
    227,226,6,126,142,137,27,251,38,134,98,191,80,103,191,0,
    147,220,225,93,156,112,92,196,76,100,136,213,143,109,47,201,
    188,192,134,141,87,249,39,91,229,94,146,54,255,146,132,91,
    19,38,220,202,187,157,48,115,47,192,80,177,201,186,104,188,
    134,18,55,121,50,154,130,21,192,211,61,140,49,228,123,212,
    94,118,159,193,131,14,154,103,141,211,104,167,219,29,114,52,
    12,223,189,229,222,248,112,233,22,63,113,243,195,59,111,191,
    255,62,251,146,15,62,126,255,206,123,239,191,183,120,171,5,
    65,243,202,218,228,88,32,121,246,79,88,221,170,159,135,57,
    15,5,109,77,154,95,184,136,165,220,231,205,67,241,234,75,
    250,247,101,69,94,249,251,210,48,44,46,72,89,198,12,231,
    217,150,158,39,31,209,180,39,228,52,133,119,83,178,65,159,
    227,244,57,41,79,91,236,215,221,115,104,206,27,78,243,211,
    67,115,14,144,242,7,152,1,43,169,53,107,178,56,118,196,
    53,148,94,193,50,94,225,95,75,175,64,230,95,101,122,200,
    241,40,121,170,98,135,32,179,163,12,112,4,59,131,242,57,
    74,167,144,182,81,59,198,109,83,167,123,203,227,216,26,72,
    242,232,193,99,12,143,50,60,129,164,14,55,28,71,30,247,
    19,78,197,84,30,183,124,2,57,28,128,41,228,112,0,78,
    34,141,3,48,141,156,7,192,12,146,54,0,167,144,179,1,
    56,141,148,13,192,25,100,108,0,206,34,97,3,112,14,137,
    24,128,243,112,105,51,130,237,137,16,151,114,136,225,190,247,
    246,252,103,43,159,60,124,129,125,231,253,171,243,175,175,252,
    38,199,181,187,61,102,216,238,110,175,215,198,237,80,46,116,
    112,10,244,103,4,172,11,157,2,233,196,134,99,87,0,22,
    24,164,179,32,11,118,136,56,69,122,13,11,38,17,106,234,
    253,93,51,163,149,129,209,17,51,58,98,70,171,3,163,163,
    102,180,14,51,152,253,177,80,214,73,69,157,50,17,202,98,
    128,144,197,8,210,140,88,249,16,9,112,132,88,32,50,89,
    139,118,188,36,124,136,8,48,87,123,80,229,89,126,225,187,
    160,145,247,190,167,92,178,23,135,15,194,152,245,197,244,228,
    29,50,7,185,182,55,29,222,183,197,35,178,6,229,36,238,
    27,37,146,193,169,116,216,68,65,102,236,121,253,84,102,168,
    169,25,239,187,71,152,225,4,239,218,186,254,92,148,23,173,
    139,86,77,70,144,115,153,205,84,140,19,66,40,71,140,174,
    108,138,126,174,114,242,59,150,39,6,51,89,142,104,175,161,
    249,30,232,40,3,92,230,110,59,237,38,197,48,162,92,48,
    82,9,234,15,196,209,252,172,164,232,85,69,209,1,130,182,
    170,48,102,100,231,63,151,71,169,125,237,16,181,246,1,106,
    89,65,222,206,115,202,217,41,64,56,28,225,223,24,142,112,
    153,246,32,4,237,79,140,181,131,97,102,234,109,205,1,166,
    126,166,164,190,103,153,250,5,231,223,7,82,238,183,196,129,
    148,27,216,105,89,69,87,105,50,69,67,15,162,180,155,15,
    131,48,91,237,218,63,18,58,192,87,138,203,70,201,250,10,
    186,164,14,153,84,129,230,128,142,190,101,148,242,93,74,232,
    194,1,57,221,30,146,156,48,119,146,18,65,191,56,68,144,
    208,127,37,65,48,34,235,28,0,122,242,200,94,227,53,127,
    87,12,207,145,170,157,28,135,126,246,167,3,123,75,234,125,
    5,33,221,198,162,127,91,152,144,85,149,47,136,255,63,229,
    234,28,153,110,210,183,157,139,58,56,189,185,50,37,122,202,
    67,84,57,156,37,11,254,233,172,184,135,142,26,23,250,152,
    98,76,179,200,254,234,161,170,150,85,88,27,40,200,194,72,
    170,34,39,10,143,54,24,216,76,163,132,153,194,139,125,152,
    81,152,173,151,157,146,172,43,98,208,234,50,210,53,85,91,
    35,227,63,12,251,251,3,66,252,23,152,97,148,153,119,142,
    98,164,201,190,34,151,229,176,89,197,194,77,83,140,209,204,
    99,83,180,110,124,44,235,181,226,142,48,91,85,239,210,44,
    74,214,221,143,74,253,232,171,203,157,33,105,10,4,229,121,
    180,16,207,251,101,95,201,167,41,72,44,109,113,169,228,99,
    229,174,229,106,209,160,9,178,15,174,126,72,165,18,46,93,
    252,213,128,138,255,223,203,149,182,24,180,157,188,112,86,195,
    7,40,8,13,131,4,4,58,127,45,142,122,192,146,235,199,
    6,185,254,189,167,174,255,215,85,163,66,106,199,53,170,191,
    253,58,2,96,27,7,75,172,255,31,214,223,228,245,183,211,
    164,240,163,36,247,188,191,251,58,2,38,7,37,0,227,119,
    72,8,188,151,253,78,39,76,130,95,195,14,214,69,124,63,
    8,60,239,239,159,66,70,185,23,174,41,91,181,199,30,151,
    172,84,54,219,175,92,239,177,15,224,208,124,74,91,125,196,
    242,76,238,33,138,65,44,23,19,163,156,88,87,248,73,59,
    116,95,51,22,76,101,221,59,5,177,66,153,22,197,22,14,
    12,200,200,12,159,61,238,125,66,252,101,223,178,77,200,25,
    171,109,34,204,50,202,252,135,82,194,249,133,129,227,161,62,
    237,170,175,217,249,142,30,37,59,127,70,149,249,225,40,31,
    48,255,108,184,194,155,43,27,236,43,109,145,61,209,140,204,
    30,244,81,6,182,238,60,169,211,160,254,100,182,201,140,8,
    202,255,4,86,171,169,171,255,234,73,203,140,147,96,104,210,
    79,111,139,123,253,106,60,159,164,229,16,80,131,157,165,195,
    98,226,8,138,229,194,108,96,169,229,29,218,179,156,231,168,
    168,248,61,136,160,114,64,44,5,138,47,56,21,138,252,216,
    100,69,236,124,1,12,67,102,20,11,138,127,196,12,167,89,
    102,117,114,190,21,235,130,156,146,53,130,207,202,41,107,156,
    254,217,25,131,74,152,235,9,72,17,97,231,99,206,130,148,
    44,149,156,140,8,105,132,120,120,230,137,196,61,36,135,103,
    40,246,46,44,45,54,146,72,207,164,190,196,255,199,236,53,
    126,102,137,39,150,248,66,136,47,164,248,194,18,107,54,18,
    96,202,86,116,85,108,132,207,101,42,226,177,242,249,252,236,
    102,93,151,221,212,142,193,210,22,149,88,84,150,203,105,27,
    26,83,59,156,98,11,105,25,245,11,202,83,25,74,154,127,
    100,58,89,244,79,120,213,143,121,175,158,89,147,98,166,199,
    107,195,169,178,90,149,141,12,124,147,203,140,123,92,127,44,
    108,189,125,233,187,164,81,113,13,176,179,107,48,143,244,167,
    163,5,111,213,69,246,231,135,102,49,15,114,193,177,243,165,
    121,176,206,15,242,153,169,126,112,66,130,125,149,126,63,241,
    232,49,207,189,69,123,224,63,81,118,11,26,232,159,33,147,
    69,124,218,231,252,45,81,84,73,81,62,124,96,214,49,70,
    87,5,83,245,52,183,37,169,241,30,159,209,62,230,202,106,
    143,235,59,123,53,49,248,92,147,75,157,234,137,31,201,96,
    188,164,255,152,25,24,21,217,31,74,172,105,66,28,90,15,
    238,28,211,119,246,184,76,26,28,231,3,189,47,101,48,137,
    162,44,218,186,72,62,22,143,45,140,58,37,171,123,13,92,
    23,77,93,224,81,213,157,189,134,184,185,207,42,114,154,158,
    37,166,80,7,238,173,51,71,78,114,201,117,84,172,89,98,
    134,122,63,215,217,195,52,20,71,237,185,52,9,221,119,140,
    93,159,205,15,36,83,223,130,246,160,186,185,115,26,203,190,
    185,114,28,194,36,237,189,43,73,126,119,229,167,35,218,72,
    40,211,45,140,179,101,123,188,30,38,110,128,30,118,196,239,
    12,199,233,154,106,13,12,208,191,247,205,242,184,172,74,247,
    99,161,203,180,236,58,244,137,134,42,217,114,173,21,167,9,
    57,142,195,62,122,202,41,132,243,252,172,62,111,76,187,73,
    224,68,137,99,138,208,224,34,206,15,24,171,62,221,113,95,
    55,211,232,3,1,126,209,224,227,36,11,219,233,122,18,125,
    22,18,46,31,71,0,235,89,218,237,12,96,27,216,198,131,
    113,194,146,163,83,133,61,118,176,167,142,114,28,215,59,220,
    62,26,86,84,112,45,246,183,87,3,255,250,40,205,154,11,
    99,121,76,76,144,99,201,231,212,146,87,38,57,17,55,1,
    14,41,142,45,14,7,57,92,194,14,19,69,13,154,43,87,
    153,156,207,134,64,14,215,63,175,145,194,226,28,70,209,99,
    177,245,207,127,67,224,180,150,69,232,168,39,28,149,85,189,
    225,64,199,156,217,224,69,167,157,198,244,219,26,55,138,196,
    50,85,89,215,170,208,121,234,157,172,171,124,220,34,246,27,
    14,26,221,14,26,120,32,21,222,112,57,124,211,224,32,42,
    178,130,163,157,44,12,186,237,176,95,159,164,37,180,55,212,
    105,67,136,102,93,152,115,135,186,86,68,15,84,232,103,243,
    110,92,48,27,35,93,126,139,73,135,107,218,215,18,88,213,
    96,174,146,20,206,134,249,204,92,113,7,79,198,253,13,252,
    205,230,137,20,46,63,8,179,226,223,48,209,187,188,109,39,
    172,26,69,84,147,214,68,229,188,125,17,135,207,220,54,229,
    9,105,203,17,26,153,182,39,248,106,204,158,150,21,123,26,
    119,91,232,33,63,110,77,73,71,54,169,167,133,168,100,32,
    72,81,155,50,239,132,237,130,45,70,187,155,101,100,7,212,
    107,55,234,117,129,85,191,189,197,39,201,125,201,168,8,179,
    187,202,177,131,10,126,14,22,109,213,235,53,191,111,182,98,
    160,180,182,100,170,251,123,104,126,252,77,242,208,125,155,16,
    255,11,102,56,201,204,171,113,88,211,144,13,107,194,154,150,
    211,175,77,203,22,31,115,245,79,96,112,169,234,211,234,64,
    60,246,115,202,85,14,29,103,185,151,209,128,28,23,229,133,
    67,229,109,117,104,6,221,112,111,161,249,45,52,40,252,184,
    223,71,131,50,135,123,79,232,183,0,212,251,0,63,68,131,
    19,35,247,19,35,20,230,42,175,127,104,103,65,144,236,54,
    102,120,25,27,93,214,72,83,244,71,67,245,241,166,221,172,
    212,171,244,177,235,53,110,205,167,86,159,212,105,209,182,143,
    151,63,212,6,121,205,168,201,187,105,122,244,221,2,208,255,
    85,47,154,233,215,11,14,184,222,239,59,255,219,204,176,219,
    9,252,98,176,124,57,204,51,132,211,198,86,62,45,177,237,
    59,172,51,71,28,86,255,245,45,194,151,13,245,189,23,76,
    232,121,103,251,43,63,124,50,201,58,253,211,97,106,35,234,
    244,167,164,57,153,36,235,197,101,153,98,35,202,29,250,250,
    78,17,230,133,11,117,117,41,128,20,17,22,200,190,181,177,
    203,175,11,224,70,143,190,190,135,27,91,115,223,109,244,26,
    28,249,96,240,210,163,28,225,222,37,243,50,213,27,151,204,
    155,84,141,221,124,14,35,187,187,151,95,116,46,95,65,201,
    180,133,44,122,103,14,190,206,217,65,92,178,147,249,201,122,
    216,250,246,220,92,79,29,3,94,218,221,233,49,223,46,237,
    68,236,169,24,142,212,207,85,150,232,103,97,150,230,24,187,
    244,146,67,93,216,19,228,245,244,211,87,123,57,172,199,70,
    232,60,36,179,203,58,130,78,63,15,214,114,88,90,38,136,
    150,20,223,191,186,130,239,115,59,207,225,247,202,163,94,143,
    168,226,208,136,79,46,93,16,229,126,142,102,17,13,40,113,
    239,154,77,145,174,110,146,129,87,70,252,77,161,109,157,218,
    162,170,36,201,167,111,74,230,75,104,88,5,32,6,197,230,
    31,27,94,179,109,99,148,138,19,238,174,208,182,110,104,213,
    118,142,181,212,139,85,215,177,96,118,142,77,169,63,214,204,
    139,51,206,204,95,214,173,38,217,251,186,53,83,171,201,103,
    173,186,180,43,35,178,193,110,178,161,63,234,52,126,156,238,
    26,151,147,228,21,108,123,220,250,31,209,211,60,241,
};

EmbeddedPython embedded_m5_util_code_formatter(
    "m5/util/code_formatter.py",
    "/mnt/hgfs/ShareShen/gem5-origin-stable-2015-9-3/src/python/m5/util/code_formatter.py",
    "m5.util.code_formatter",
    data_m5_util_code_formatter,
    3982,
    11516);

} // anonymous namespace
