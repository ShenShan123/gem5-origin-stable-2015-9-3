#include "sim/init.hh"

namespace {

const uint8_t data_m5_trace[] = {
    120,156,181,81,77,75,196,48,16,157,180,221,234,194,226,69,
    240,38,136,32,244,210,141,31,236,65,16,17,148,61,238,33,
    245,212,139,212,54,182,93,210,116,73,167,130,119,255,183,206,
    4,217,213,171,104,72,134,151,97,230,189,121,73,9,95,43,
    160,115,119,2,48,60,16,168,104,11,48,0,249,22,11,200,
    133,199,1,152,0,186,16,242,16,186,8,242,8,68,21,194,
    59,149,78,160,138,60,136,185,33,75,38,68,212,126,208,90,
    37,76,142,49,133,126,196,205,136,30,182,181,237,157,46,191,
    235,223,179,254,33,215,210,22,176,14,96,29,194,83,196,178,
    89,34,40,191,242,164,200,97,89,152,65,227,62,19,89,212,
    206,22,198,167,209,21,165,198,136,80,249,90,56,220,35,160,
    109,241,108,116,149,176,202,46,12,75,10,178,179,40,155,250,
    101,144,89,83,56,157,53,218,202,90,119,139,180,119,109,221,
    218,116,64,110,77,47,207,47,22,233,117,122,37,7,87,202,
    205,27,54,189,149,221,66,122,173,57,221,89,165,106,7,174,
    61,101,106,246,2,226,87,214,120,242,71,55,106,21,18,80,
    124,83,156,87,241,143,225,255,220,65,188,125,167,179,157,129,
    132,125,249,65,252,88,35,182,6,15,190,61,248,220,183,43,
    174,85,108,65,249,250,233,255,78,202,63,126,211,245,213,104,
    244,237,49,115,115,98,38,102,193,81,48,13,63,1,23,203,
    147,32,
};

EmbeddedPython embedded_m5_trace(
    "m5/trace.py",
    "/mnt/hgfs/ShareShen/gem5-origin-stable-2015-9-3/src/python/m5/trace.py",
    "m5.trace",
    data_m5_trace,
    322,
    718);

} // anonymous namespace
