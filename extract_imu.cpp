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

#define	SHOW_VIDEO_FRAMERATE		1
#define	SHOW_PAYLOAD_TIME			1
#define	SHOW_ALL_PAYLOADS			0
#define SHOW_GPMF_STRUCTURE			1
#define	SHOW_PAYLOAD_INDEX			0
#define	SHOW_SCALED_DATA			1
#define	SHOW_THIS_FOUR_CC			STR2FOURCC("ACCL")
#define SHOW_COMPUTED_SAMPLERATES	1



extern void PrintGPMF(GPMF_stream* ms);

void printHelp(char* name)
{
	printf("usage: %s <file_with_GPMF> <optional features>\n", name);
	printf("       -a - %s all payloads\n", SHOW_ALL_PAYLOADS ? "disable" : "show");
	printf("       -g - %s GPMF structure\n", SHOW_GPMF_STRUCTURE ? "disable" : "show");
	printf("       -i - %s index of the payload\n", SHOW_PAYLOAD_INDEX ? "disable" : "show");
	printf("       -s - %s scaled data\n", SHOW_SCALED_DATA ? "disable" : "show");
	printf("       -c - %s computed sample rates\n", SHOW_COMPUTED_SAMPLERATES ? "disable" : "show");
	printf("       -v - %s video framerate\n", SHOW_VIDEO_FRAMERATE ? "disable" : "show");
	printf("       -t - %s time of the payload\n", SHOW_PAYLOAD_TIME ? "disable" : "show");
	printf("       -fWXYZ - show only this fourCC , e.g. -f%c%c%c%c (default) just -f for all\n", PRINTF_4CC(SHOW_THIS_FOUR_CC));
	printf("       -h - this help\n");
	printf("       \n");
	printf("       ver 2.0\n");
}

int main(int argc, char* argv[])
{
	GPMF_ERR ret = GPMF_OK;
	GPMF_stream metadata_stream, * ms = &metadata_stream;
	double metadatalength;
	uint32_t payloadsize;
	uint32_t* payload = NULL; //buffer to store GPMF samples from the MP4.

	uint32_t show_all_payloads = SHOW_ALL_PAYLOADS;
	uint32_t show_gpmf_structure = SHOW_GPMF_STRUCTURE;
	uint32_t show_payload_index = SHOW_PAYLOAD_INDEX;
	uint32_t show_scaled_data = SHOW_SCALED_DATA;
	uint32_t show_computed_samplerates = SHOW_COMPUTED_SAMPLERATES;
	uint32_t show_video_framerate = SHOW_VIDEO_FRAMERATE;
	uint32_t show_payload_time = SHOW_PAYLOAD_TIME;
	uint32_t show_this_four_cc = SHOW_THIS_FOUR_CC;

	// get file return data
	if (argc < 2)
	{
		printHelp(argv[0]);
		return -1;
	}

#if 1 // Search for GPMF Track
	size_t mp4 = OpenMP4Source(argv[1], MOV_GPMF_TRAK_TYPE, MOV_GPMF_TRAK_SUBTYPE);
#else // look for a global GPMF payload in the moov header, within 'udta'
	size_t mp4 = OpenMP4SourceUDTA(argv[1]);  //Search for GPMF payload with MP4's udta
#endif
	if (mp4 == 0)
	{
		printf("error: %s is an invalid MP4/MOV or it has no GPMF data\n\n", argv[1]);

		printHelp(argv[0]);
		return -1;
	}

	metadatalength = GetDuration(mp4);

	//If the GPMF streams are using (non-zero) timestamps, which stream should time zero be relative to.
	//SetTimeBaseStream(mp4, STR2FOURCC("SHUT"));

	if (metadatalength > 0.0)
	{
		uint32_t index, payloads = GetNumberPayloads(mp4);
		//		printf("found %.2fs of metadata, from %d payloads, within %s\n", metadatalength, payloads, argv[1]);

		uint32_t fr_num, fr_dem;
		uint32_t frames = GetVideoFrameRateAndCount(mp4, &fr_num, &fr_dem);

		if (frames)
		{
			printf("VIDEO FRAMERATE:\n  %.3f with %d frames\n", (float)fr_num / (float)fr_dem, frames);
		}

		for (index = 0; index < payloads; index++)
		{
			double in = 0.0, out = 0.0; //times
			payloadsize = GetPayloadSize(mp4, index);
			payload = GetPayload(mp4, payload, index);
			if (payload == NULL)
				goto cleanup;

			ret = GetPayloadTime(mp4, index, &in, &out);
			if (ret != GPMF_OK)
				goto cleanup;

			ret = GPMF_Init(ms, payload, payloadsize);
			if (ret != GPMF_OK)
				goto cleanup;

			if (show_payload_time)
				if (show_gpmf_structure || show_payload_index || show_scaled_data)
					if (show_all_payloads || index == 0)
						printf("PAYLOAD TIME:\n  %.3f to %.3f seconds\n", in, out);

			if (show_gpmf_structure)
			{
				if (show_all_payloads || index == 0)
				{
					printf("GPMF STRUCTURE:\n");
					// Output (printf) all the contained GPMF data within this payload
					ret = GPMF_Validate(ms, GPMF_RECURSE_LEVELS); // optional
					if (GPMF_OK != ret)
					{
						printf("Invalid GPMF Structure\n");
						goto cleanup;
					}

					GPMF_ResetState(ms);
					do
					{
						printf("  ");
						PrintGPMF(ms);  // printf current GPMF KLV
;
					} while (GPMF_OK == GPMF_Next(ms, GPMF_RECURSE_LEVELS));
					GPMF_ResetState(ms);
				}
			}

			if (show_scaled_data)
			{
				if (show_all_payloads || index == 0)
				{
					printf("SCALED DATA:\n");
					while (GPMF_OK == GPMF_FindNext(ms, STR2FOURCC("STRM"), GPMF_RECURSE_LEVELS)) //GoPro Hero5/6/7 Accelerometer)
					{
						if (GPMF_VALID_FOURCC(show_this_four_cc))
						{
							if (GPMF_OK != GPMF_FindNext(ms, show_this_four_cc, GPMF_RECURSE_LEVELS))
								continue;
						}
						else
						{
							ret = GPMF_SeekToSamples(ms);
							if (GPMF_OK != ret) break;
						}

						char* rawdata = (char*)GPMF_RawData(ms);
						uint32_t key = GPMF_Key(ms);
						GPMF_SampleType type = GPMF_Type(ms);
						uint32_t samples = GPMF_Repeat(ms);
						uint32_t elements = GPMF_ElementsInStruct(ms);
						uint32_t buffersize = samples * elements * sizeof(double);
						GPMF_stream find_stream;
						double* ptr, * tmpbuffer = (double*)malloc(buffersize);

#define MAX_UNITS	64
#define MAX_UNITLEN	8
						char units[MAX_UNITS][MAX_UNITLEN] = { "" };
						uint32_t unit_samples = 1;

						char complextype[MAX_UNITS] = { "" };
						uint32_t type_samples = 1;

						if (tmpbuffer && samples)
						{
							uint32_t i, j;

							//Search for any units to display
							GPMF_CopyState(ms, &find_stream);
							if (GPMF_OK == GPMF_FindPrev(&find_stream, GPMF_KEY_SI_UNITS, GPMF_CURRENT_LEVEL) ||
							    GPMF_OK == GPMF_FindPrev(&find_stream, GPMF_KEY_UNITS, GPMF_CURRENT_LEVEL))
							{
								char* data = (char*)GPMF_RawData(&find_stream);
								uint32_t ssize = GPMF_StructSize(&find_stream);
								if (ssize > MAX_UNITLEN - 1) ssize = MAX_UNITLEN - 1;
								unit_samples = GPMF_Repeat(&find_stream);
								for (i = 0; i < unit_samples && i < MAX_UNITS; i++)
								{
									memcpy(units[i], data, ssize);
									units[i][ssize] = 0;
									data += ssize;
								}
							}

							//Search for TYPE if Complex
							GPMF_CopyState(ms, &find_stream);
							type_samples = 0;
							if (GPMF_OK == GPMF_FindPrev(&find_stream, GPMF_KEY_TYPE, GPMF_CURRENT_LEVEL))
							{
								char* data = (char*)GPMF_RawData(&find_stream);
								uint32_t ssize = GPMF_StructSize(&find_stream);
								if (ssize > MAX_UNITLEN - 1) ssize = MAX_UNITLEN - 1;
								type_samples = GPMF_Repeat(&find_stream);

								for (i = 0; i < type_samples && i < MAX_UNITS; i++)
								{
									complextype[i] = data[i];
								}
							}

							//GPMF_FormattedData(ms, tmpbuffer, buffersize, 0, samples); // Output data in LittleEnd, but no scale
							if (GPMF_OK == GPMF_ScaledData(ms, tmpbuffer, buffersize, 0, samples, GPMF_TYPE_DOUBLE))//Output scaled data as floats
							{

								ptr = tmpbuffer;
								int pos = 0;
								for (i = 0; i < samples; i++)
								{
									printf("  %c%c%c%c ", PRINTF_4CC(key));

									for (j = 0; j < elements; j++)
									{
										if (type == GPMF_TYPE_STRING_ASCII)
										{
											printf("%c", rawdata[pos]);
											pos++;
											ptr++;
										}
										else if (type_samples == 0) //no TYPE structure
											printf("%.3f%s, ", *ptr++, units[j % unit_samples]);
										else if (complextype[j] != 'F')
										{
											printf("%.3f%s, ", *ptr++, units[j % unit_samples]);
											pos += GPMF_SizeofType((GPMF_SampleType)complextype[j]);
										}
										else if (type_samples && complextype[j] == GPMF_TYPE_FOURCC)
										{
											ptr++;
											printf("%c%c%c%c, ", rawdata[pos], rawdata[pos + 1], rawdata[pos + 2], rawdata[pos + 3]);
											pos += GPMF_SizeofType((GPMF_SampleType)complextype[j]);
										}
									}


									printf("\n");
								}
							}
							free(tmpbuffer);
						}
					}
					GPMF_ResetState(ms);
				}
			}

			GPMF_Free(ms);
		}

		if (show_computed_samplerates)
		{
			printf("COMPUTED SAMPLERATES:\n");
			// Find all the available Streams and compute they sample rates
			while (GPMF_OK == GPMF_FindNext(ms, GPMF_KEY_STREAM, GPMF_RECURSE_LEVELS))
			{
//				if (GPMF_OK == GPMF_SeekToSamples(ms)) //find the last FOURCC within the stream
//				{
					double start, end;
					uint32_t fourcc = STR2FOURCC("ACCL");
					if (GPMF_OK != GPMF_FindNext(ms, fourcc, GPMF_RECURSE_LEVELS))
						continue;

					double rate = GetGPMFSampleRate(mp4, fourcc, GPMF_SAMPLE_RATE_PRECISE, &start, &end);// GPMF_SAMPLE_RATE_FAST);
					printf("  %c%c%c%c sampling rate = %fHz (time %f to %f)\",\n", PRINTF_4CC(fourcc), rate, start, end);
//				}
			}

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
