#include <string>
#include <cstring>

#include "GPMF_parser.h"
#include "GPMF_mp4reader.h"

using namespace std;

int main(int argc, char *argv[])
{
    std::string filename = "/home/bjoshi/gopro9/GX010037.MP4";
    char *video;
    strcpy(video, filename.c_str());

    GPMF_ERR ret = GPMF_OK;
    GPMF_stream metadata_stream, *ms = &metadata_stream;
    uint32_t *payload = NULL;
    uint32_t payloadsize = 0;
    size_t payloadres = 0;

    // size_t mp4handle = OpenMP4Source(video, MOV_VIDE_TRAK_TYPE)
}