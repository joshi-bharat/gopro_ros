/*! @file GPMF_demo.c
 *
 *  @brief Demo to extract GPMF from an MP4
 *
 *  @version 2.0.0
 *
 *  (C) Copyright 2017-2020 GoPro Inc (http://gopro.com/).
 *
 *  Licensed under either:
 *  - Apache License, Version 2.0, http://www.apache.org/licenses/LICENSE-2.0
 *  - MIT license, http://opensource.org/licenses/MIT
 *  at your option.
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "GPMF_parser.h"
#include "GPMF_mp4reader.h"


extern void PrintGPMF(GPMF_stream* ms);


int main(int argc, char* argv[])
{
	GPMF_ERR ret = GPMF_OK;
	GPMF_stream metadata_stream, * ms = &metadata_stream;
	double metadatalength;
	uint32_t payloadsize;
	uint32_t* payload = NULL; //buffer to store GPMF samples from the MP4.


    char * video_file = argv[1];

	size_t mp4 = OpenMP4Source(video_file, MOV_GPMF_TRAK_TYPE, MOV_GPMF_TRAK_SUBTYPE);

	if (mp4 == 0)
	{
		printf("error: %s is an invalid MP4/MOV or it has no GPMF data\n\n", argv[1]);
		return -1;
	}

	metadatalength = GetDuration(mp4);


	if (metadatalength > 0.0)
	{
		uint32_t index, payloads = GetNumberPayloads(mp4);
				printf("found %.2fs of metadata, from %d payloads, within %s\n", metadatalength, payloads, video_file);

		uint32_t fr_num, fr_dem;
		uint32_t frames = GetVideoFrameRateAndCount(mp4, &fr_num, &fr_dem);

		if (frames)
		{
			printf("VIDEO FRAMERATE:\n  %.3f with %d frames\n", (float)fr_num / (float)fr_dem, frames);
		}


        for (index = 0; index < payloads; index++) {
            double in = 0.0, out = 0.0; //times
            payloadsize = GetPayloadSize(mp4, index);
            payload = GetPayload(mp4, payload, index);
            if (payload == NULL)
                goto cleanup;


            GPMF_Free(ms);

        }

        cleanup:
        if (ms) GPMF_Free(ms);
        if (payload) FreePayload(payload); payload = NULL;
        CloseSource(mp4);

	}

	if (ret != 0)
		printf("GPMF data has corruption\n");

	return (int)ret;
}
