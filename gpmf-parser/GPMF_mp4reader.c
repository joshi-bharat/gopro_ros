/*! @file mp4reader.c
*
*  @brief Way Too Crude MP4|MOV reader
*
*  @version 1.8.0
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

/* This is not an elegant MP4 parser, only used to help demonstrate extraction of GPMF */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "include/GPMF_mp4reader.h"
#include "include/GPMF_common.h"

#define PRINT_MP4_STRUCTURE		1

#ifdef _WINDOWS
#define LONGTELL	_ftelli64
#else
#define LONGTELL	ftell
#endif


uint32_t GetNumberPayloads(size_t handle)
{
	mp4object *mp4 = (mp4object *)handle;

	if (mp4)
	{
		return mp4->indexcount;
	}

	return 0;
}

uint32_t *GetPayload(size_t handle, uint32_t *lastpayload, uint32_t index)
{
	mp4object *mp4 = (mp4object *)handle;
	if (mp4 == NULL) return NULL;

	uint32_t *MP4buffer = NULL;
	if (index < mp4->indexcount && mp4->mediafp)
	{
		if ((mp4->filesize >= mp4->metaoffsets[index]+mp4->metasizes[index]) && (mp4->metasizes[index] > 0))
		{
			MP4buffer = (uint32_t *)realloc((void *)lastpayload, mp4->metasizes[index]);

			if (MP4buffer)
			{
#ifdef _WINDOWS
				_fseeki64(mp4->mediafp, (__int64) mp4->metaoffsets[index], SEEK_SET);
#else
				fseeko(mp4->mediafp, (off_t) mp4->metaoffsets[index], SEEK_SET);
#endif
				fread(MP4buffer, 1, mp4->metasizes[index], mp4->mediafp);
				mp4->filepos = mp4->metaoffsets[index] + mp4->metasizes[index];
				return MP4buffer;
			}
		}
	}
	if (lastpayload)
		free(lastpayload);

	return NULL;
}


uint32_t WritePayload(size_t handle, uint32_t *payload, uint32_t payloadsize, uint32_t index)
{
	mp4object* mp4 = (mp4object*)handle;
	if (mp4 == NULL) return 0;

	if (index < mp4->indexcount && mp4->mediafp)
	{
		if ((mp4->filesize >= mp4->metaoffsets[index] + mp4->metasizes[index]) && mp4->metasizes[index] == payloadsize)
		{
#ifdef _WINDOWS
			_fseeki64(mp4->mediafp, (__int64)mp4->metaoffsets[index], SEEK_SET);
#else
			fseeko(mp4->mediafp, (off_t)mp4->metaoffsets[index], SEEK_SET);
#endif
			fwrite(payload, 1, payloadsize, mp4->mediafp);
			mp4->filepos = mp4->metaoffsets[index] + payloadsize;
			return payloadsize;
		}
	}

	return 0;
}



void LongSeek(mp4object *mp4, int64_t offset)
{
	if (mp4 && offset)
	{
		if (mp4->filepos + offset < mp4->filesize)
		{
#ifdef _WINDOWS
			_fseeki64(mp4->mediafp, (__int64)offset, SEEK_CUR);
#else
			fseeko(mp4->mediafp, (off_t)offset, SEEK_CUR);
#endif
			mp4->filepos += offset;
		}
		else
		{
			mp4->filepos = mp4->filesize;
		}
	}
}

void FreePayload(uint32_t *lastpayload)
{
	if (lastpayload)
		free(lastpayload);
}


uint32_t GetPayloadSize(size_t handle, uint32_t index)
{
	mp4object *mp4 = (mp4object *)handle;
	if (mp4 == NULL) return 0;

	if (mp4->metasizes && mp4->metasize_count > index)
		return mp4->metasizes[index] & (uint32_t)~0x3;  //All GPMF payloads are 32-bit aligned and sized

	return 0;
}


#define MAX_NEST_LEVEL	20

size_t OpenMP4Source(char *filename, uint32_t traktype, uint32_t traksubtype)  //RAW or within MP4
{
	mp4object *mp4 = (mp4object *)malloc(sizeof(mp4object));
	if (mp4 == NULL) return 0;

	memset(mp4, 0, sizeof(mp4object));

#ifdef _WINDOWS
	struct _stat64 mp4stat;
	_stat64(filename, &mp4stat);
#else
	struct stat mp4stat;
	stat(filename, &mp4stat);
#endif
	mp4->filesize = (uint64_t) mp4stat.st_size;
//	printf("filesize = %ld\n", mp4->filesize);
	if (mp4->filesize < 64) 
	{
		free(mp4);
		return 0;
	}

#ifdef _WINDOWS
	fopen_s(&mp4->mediafp, filename, "rb+");
#else
	mp4->mediafp = fopen(filename, "rb+");
#endif

	if (mp4->mediafp)
	{
		uint32_t qttag, qtsize32, skip, type = 0, subtype = 0, num;
		size_t len;
		int32_t nest = 0;
		uint64_t nestsize[MAX_NEST_LEVEL] = { 0 };
		uint64_t lastsize = 0, qtsize;


		do
		{
			len = fread(&qtsize32, 1, 4, mp4->mediafp);
			len += fread(&qttag, 1, 4, mp4->mediafp);
			mp4->filepos += len;
			if (len == 8 && mp4->filepos < mp4->filesize)
			{
				if (mp4->filepos == 8 && qttag != MAKEID('f', 't', 'y', 'p'))
				{
					CloseSource((size_t)mp4);
					mp4 = NULL;
					break;
				}

				if (!GPMF_VALID_FOURCC(qttag))
				{
					CloseSource((size_t)mp4);
					mp4 = NULL;
					break;
				}

				qtsize32 = BYTESWAP32(qtsize32);

				if (qtsize32 == 1) // 64-bit Atom
				{
					len = fread(&qtsize, 1, 8, mp4->mediafp);
					mp4->filepos += len;
					qtsize = BYTESWAP64(qtsize) - 8;
				}
				else
					qtsize = qtsize32;

				if(qtsize-len > (mp4->filesize - mp4->filepos))  // not parser truncated files.
				{
					CloseSource((size_t)mp4);
					mp4 = NULL;
					break;
				}

				nest++;

				if (qtsize < 8) break;
				if (nest >= MAX_NEST_LEVEL) break;

				nestsize[nest] = qtsize;
				lastsize = qtsize;

#if PRINT_MP4_STRUCTURE	

				for (int i = 1; i < nest; i++) printf("    ");
				printf("%c%c%c%c (%lld)\n", (qttag & 0xff), ((qttag >> 8) & 0xff), ((qttag >> 16) & 0xff), ((qttag >> 24) & 0xff), qtsize);

				if (qttag == MAKEID('m', 'd', 'a', 't') ||
					qttag == MAKEID('f', 't', 'y', 'p') ||
					qttag == MAKEID('u', 'd', 't', 'a') ||
					qttag == MAKEID('f', 'r', 'e', 'e'))
				{
					LongSeek(mp4, qtsize - 8);

					NESTSIZE(qtsize);

					continue;
				}
#endif
				if (qttag != MAKEID('m', 'o', 'o', 'v') && //skip over all but these atoms
					qttag != MAKEID('m', 'v', 'h', 'd') &&
					qttag != MAKEID('t', 'r', 'a', 'k') &&
					qttag != MAKEID('m', 'd', 'i', 'a') &&
					qttag != MAKEID('m', 'd', 'h', 'd') &&
					qttag != MAKEID('m', 'i', 'n', 'f') &&
					qttag != MAKEID('g', 'm', 'i', 'n') &&
					qttag != MAKEID('d', 'i', 'n', 'f') &&
					qttag != MAKEID('a', 'l', 'i', 's') &&
					qttag != MAKEID('s', 't', 's', 'd') &&
					qttag != MAKEID('s', 't', 'b', 'l') &&
					qttag != MAKEID('s', 't', 't', 's') &&
					qttag != MAKEID('s', 't', 's', 'c') &&
					qttag != MAKEID('s', 't', 's', 'z') &&
					qttag != MAKEID('s', 't', 'c', 'o') &&
					qttag != MAKEID('c', 'o', '6', '4') &&
					qttag != MAKEID('h', 'd', 'l', 'r') &&
					qttag != MAKEID('e', 'd', 't', 's') &&
					qttag != MAKEID('t', 'r', 'e', 'f'))
//					&& qttag != MAKEID('t', 'k' ,'h' ,'d'))

				{
					LongSeek(mp4, qtsize - 8);

					NESTSIZE(qtsize);
				}
				else if (qttag == MAKEID('m', 'v', 'h', 'd')) //mvhd  movie header
				{
					len = fread(&skip, 1, 4, mp4->mediafp);
					len += fread(&mp4->movie_creation_time, 1, 4, mp4->mediafp);
					mp4->movie_creation_time = BYTESWAP32(mp4->movie_creation_time);
					len += fread(&skip, 1, 4, mp4->mediafp);
					len += fread(&mp4->clockdemon, 1, 4, mp4->mediafp); mp4->clockdemon = BYTESWAP32(mp4->clockdemon);
					len += fread(&mp4->clockcount, 1, 4, mp4->mediafp); mp4->clockcount = BYTESWAP32(mp4->clockcount);

					mp4->filepos += len;
					LongSeek(mp4, qtsize - 8 - len); // skip over mvhd

					NESTSIZE(qtsize);
				}
				else if (qttag == MAKEID('t', 'r', 'a', 'k')) //trak header
				{

					if (mp4->trak_num+1 < MAX_TRACKS)
						mp4->trak_num++;

					NESTSIZE(qtsize);
					if (PRINT_MP4_STRUCTURE)
						printf("***********New Track**************\n");
				}
				else if(qttag == MAKEID('t', 'r', 'e', 'f')){
					uint32_t tref_type;
					len = fread(&skip, 1, 4, mp4->mediafp);
					len += fread(&tref_type, 1, 4, mp4->mediafp);

					if(PRINT_MP4_STRUCTURE)
						printf("tref_type: %c%c%c%c", PRINTF_4CC(tref_type));
					mp4->filepos += len;
					LongSeek(mp4, qtsize - 8 - len); // skip over edts

					NESTSIZE(qtsize);
				}
				else if (qttag == MAKEID('m', 'd', 'h', 'd')) //mdhd  media header
				{
					media_header md;
					len = fread(&md, 1, sizeof(md), mp4->mediafp);
					if (len == sizeof(md))
					{
						md.creation_time = BYTESWAP32(md.creation_time);
						md.modification_time = BYTESWAP32(md.modification_time);
						md.time_scale = BYTESWAP32(md.time_scale);
						md.duration = BYTESWAP32(md.duration);

						mp4->trak_clockdemon = md.time_scale;
						mp4->trak_clockcount = md.duration;

						if (mp4->trak_clockdemon == 0 || mp4->trak_clockcount == 0)
						{
							CloseSource((size_t)mp4);
							mp4 = NULL;
							break;
						}

//						if (mp4->videolength == 0.0) // Get the video length from the first track
//						{
//							mp4->videolength = (float)((double)mp4->trak_clockcount / (double)mp4->trak_clockdemon);
//						}
					}

						mp4->filepos += len;
						LongSeek(mp4, qtsize - 8 - len); // skip over mvhd

						NESTSIZE(qtsize);
					}
				else if (qttag == MAKEID('h', 'd', 'l', 'r')) //hldr
				{
					uint32_t temp;
					len = fread(&skip, 1, 4, mp4->mediafp);
					len += fread(&skip, 1, 4, mp4->mediafp);
					len += fread(&temp, 1, 4, mp4->mediafp);  // type will be 'meta' for the correct trak.

					if (temp != MAKEID('a', 'l', 'i', 's') && temp != MAKEID('u', 'r', 'l', ' '))
						type = temp;

					if(PRINT_MP4_STRUCTURE)
						printf("type: %c%c%c%c\n", PRINTF_4CC(type));

					mp4->filepos += len;
					LongSeek(mp4, qtsize - 8 - len); // skip over hldr

					NESTSIZE(qtsize);

				}
				else if (qttag == MAKEID('e', 'd', 't', 's')) //edit list
				{
					uint32_t elst,temp,readnum,i;
					len = fread(&skip, 1, 4, mp4->mediafp);
					len += fread(&elst, 1, 4, mp4->mediafp);
					if (elst == MAKEID('e', 'l', 's', 't'))
					{
						len += fread(&temp, 1, 4, mp4->mediafp);
						if (temp == 0)
						{
							len += fread(&readnum, 1, 4, mp4->mediafp);
							readnum = BYTESWAP32(readnum);
							if (readnum <= (qtsize / 12))
							{
								uint32_t segment_duration; //integer that specifies the duration of this edit segment in units of the movie�s time scale.
								uint32_t segment_mediaTime; //integer containing the starting time within the media of this edit segment(in media timescale units).If this field is set to �1, it is an empty edit.The last edit in a track should never be an empty edit.Any difference between the movie�s duration and the track�s duration is expressed as an implicit empty edit.
								uint32_t segment_mediaRate; //point number that specifies the relative rate at which to play the media corresponding to this edit segment.This rate value cannot be 0 or negative.
								for (i = 0; i < readnum; i++)
								{
									len += fread(&segment_duration, 1, 4, mp4->mediafp);
									len += fread(&segment_mediaTime, 1, 4, mp4->mediafp);
									len += fread(&segment_mediaRate, 1, 4, mp4->mediafp);

									segment_duration = BYTESWAP32(segment_duration);  // in MP4 clock base
									segment_mediaTime = BYTESWAP32(segment_mediaTime); // in trak clock base
									segment_mediaRate = BYTESWAP32(segment_mediaRate); // Fixed-point 65536 = 1.0X

									if (segment_mediaTime == 0xffffffff) // the segment_duration for blanked time
										mp4->trak_edit_list_offsets[mp4->trak_num] += (int32_t)segment_duration;  //samples are delay, data starts after presentation time zero.
									else if (i == 0) // If the first editlst starts after zero, the track is offset by this time (time before presentation time zero.)
										mp4->trak_edit_list_offsets[mp4->trak_num] -= (int32_t)((double)segment_mediaTime/(double)mp4->trak_clockdemon*(double)mp4->clockdemon); //convert to MP4 clock base.
								}
								if (type == traktype) // GPMF metadata
								{
									mp4->metadataoffset_clockcount = mp4->trak_edit_list_offsets[mp4->trak_num]; //leave in MP4 clock base
								}
							}
						}
					}
					mp4->filepos += len;
					LongSeek(mp4, qtsize - 8 - len); // skip over edts

					NESTSIZE(qtsize);
				}
				else if (qttag == MAKEID('s', 't', 's', 'd')) //read the sample decription to determine the type of metadata
				{
					if (type == traktype) //like meta
					{
						len = fread(&skip, 1, 4, mp4->mediafp);
						len += fread(&skip, 1, 4, mp4->mediafp);
						len += fread(&skip, 1, 4, mp4->mediafp);
						len += fread(&subtype, 1, 4, mp4->mediafp);  // type will be 'meta' for the correct trak.
						if(PRINT_MP4_STRUCTURE)
							printf("subtype: %c%c%c%c\n",PRINTF_4CC(subtype));
						if (len == 16)
						{
							if (subtype != traksubtype) // not MP4 metadata
							{
								type = 0; // MP4
							}
						}
						mp4->filepos += len;
						LongSeek(mp4, qtsize - 8 - len); // skip over stsd
					}
						else
							LongSeek(mp4, qtsize - 8);

						NESTSIZE(qtsize);
					}
				else if (qttag == MAKEID('s', 't', 's', 'c')) // metadata stsc - offset chunks
				{
					if (type == traktype) // meta
					{
						len = fread(&skip, 1, 4, mp4->mediafp);
						len += fread(&num, 1, 4, mp4->mediafp);

						num = BYTESWAP32(num);
						if (num <= (qtsize/sizeof(SampleToChunk)))
						{
							mp4->metastsc_count = num;
							if (mp4->metastsc)
							{
								free(mp4->metastsc);
								mp4->metastsc = 0;
							}
							if (num > 0 && qtsize > (num * sizeof(SampleToChunk)))
							{
								mp4->metastsc = (SampleToChunk *)malloc(num * sizeof(SampleToChunk));
								if (mp4->metastsc)
								{
									len += fread(mp4->metastsc, 1, num * sizeof(SampleToChunk), mp4->mediafp);

									do
									{
										num--;
										mp4->metastsc[num].chunk_num = BYTESWAP32(mp4->metastsc[num].chunk_num);
										mp4->metastsc[num].samples = BYTESWAP32(mp4->metastsc[num].samples);
										mp4->metastsc[num].id = BYTESWAP32(mp4->metastsc[num].id);
									} while (num > 0);
								}
							}
							else
							{
								//size of null
								CloseSource((size_t)mp4);
								mp4 = NULL;
								break;
							}
						}
						mp4->filepos += len;
						LongSeek(mp4, qtsize - 8 - len); // skip over stsx
					}
					else
						LongSeek(mp4, qtsize - 8);

					NESTSIZE(qtsize);
				}
				else if (qttag == MAKEID('s', 't', 's', 'z')) // metadata stsz - sizes
				{
					if (type == traktype) // meta
					{
						uint32_t equalsamplesize;

						len = fread(&skip, 1, 4, mp4->mediafp);
						len += fread(&equalsamplesize, 1, 4, mp4->mediafp);
						len += fread(&num, 1, 4, mp4->mediafp);

						num = BYTESWAP32(num);
						// if equalsamplesize != 0, it is the size of all the samples and the length should be 20 (size,fourcc,flags,samplesize,samplecount)
						if ((num <= (qtsize/sizeof(uint32_t))) || (equalsamplesize != 0 && qtsize == 20))
						{
							if (mp4->metasizes)
							{
								free(mp4->metasizes);
								mp4->metasizes = 0;
							}
							if(num > 0 && qtsize > (num * 4))
							{
								mp4->metasize_count = num;
								mp4->metasizes = (uint32_t *)malloc(num * 4);
								if (mp4->metasizes)
								{
									if (equalsamplesize == 0)
									{
										len += fread(mp4->metasizes, 1, num * 4, mp4->mediafp);
										do
										{
											num--;
											mp4->metasizes[num] = BYTESWAP32(mp4->metasizes[num]);
										} while (num > 0);
									}
									else
									{
										equalsamplesize = BYTESWAP32(equalsamplesize);
										do
										{
											num--;
											mp4->metasizes[num] = equalsamplesize;
										} while (num > 0);
									}
								}
							}
							else
							{
								//size of null
								CloseSource((size_t)mp4);
								mp4 = NULL;
								break;
							}
						}
						mp4->filepos += len;
						LongSeek(mp4, qtsize - 8 - len); // skip over stsz
					}
					else
						LongSeek(mp4, qtsize - 8);

					NESTSIZE(qtsize);
				}
				else if (qttag == MAKEID('s', 't', 'c', 'o')) // metadata stco - offsets
				{
					if (type == traktype) // meta
					{
						len = fread(&skip, 1, 4, mp4->mediafp);
						len += fread(&num, 1, 4, mp4->mediafp);
						num = BYTESWAP32(num);
						if (num <= (qtsize/sizeof(uint32_t)))
						{
							mp4->metastco_count = num;

							if (mp4->metastsc_count > 0 && num != mp4->metasize_count)
							{
								if (mp4->metaoffsets)
								{
									free(mp4->metaoffsets);
									mp4->metaoffsets = 0;
								}
								if(num > 0 && qtsize > (num * 4))
								{
									mp4->metaoffsets = (uint64_t *)malloc(num * 8);
									if (mp4->metaoffsets)
									{
										uint32_t *metaoffsets32 = NULL;
										metaoffsets32 = (uint32_t *)malloc(num * 4);
										if (metaoffsets32)
										{
											uint64_t fileoffset = 0;
											int stsc_pos = 0;
											int stco_pos = 0;
											int repeat = 1;
											len += fread(metaoffsets32, 1, num * 4, mp4->mediafp);
											do
											{
												num--;
												metaoffsets32[num] = BYTESWAP32(metaoffsets32[num]);
											} while (num > 0);

											mp4->metaoffsets[0] = fileoffset = metaoffsets32[stco_pos];
											num = 1;
											while (num < mp4->metastco_count)
											{
												if ((uint32_t)repeat == mp4->metastsc[stsc_pos].samples)
												{
													if ((uint32_t)stco_pos + 1 < mp4->metastco_count)
													{
														stco_pos++;
														fileoffset = (uint64_t)metaoffsets32[stco_pos];
													}
													else
													{
														fileoffset += (uint64_t)mp4->metasizes[num - 1];
													}
													if ((uint32_t)stsc_pos + 1 < mp4->metastsc_count)
														if (mp4->metastsc[stsc_pos + 1].chunk_num == (uint32_t)stco_pos + 1)
															stsc_pos++;

													repeat = 1;
												}
												else
												{
													fileoffset += (uint64_t)mp4->metasizes[num - 1];
													repeat++;
												}

												mp4->metaoffsets[num] = fileoffset;
												//int delta = metaoffsets[num] - metaoffsets[num - 1];
												//printf("%3d:%08x, delta = %08x\n", num, (int)fileoffset, delta);

												num++;
											}

											if (mp4->metastsc) free(mp4->metastsc);
											mp4->metastsc = NULL;
											mp4->metastsc_count = 0;

											free(metaoffsets32);
										}
									}
								}
								else
								{
									//size of null
									CloseSource((size_t)mp4);
									mp4 = NULL;
									break;
								}
							}
							else
							{
								if (mp4->metaoffsets)
								{
									free(mp4->metaoffsets);
									mp4->metaoffsets = 0;
								}
								if (num > 0 && qtsize > (num * 4))
								{
									mp4->metaoffsets = (uint64_t *)malloc(num * 8);
									if (mp4->metaoffsets)
									{
										uint32_t *metaoffsets32 = NULL;
										metaoffsets32 = (uint32_t *)malloc(num * 4);
										if (metaoffsets32)
										{
											size_t readlen = fread(metaoffsets32, 1, num * 4, mp4->mediafp);
											len += readlen;
											do
											{
												num--;
												mp4->metaoffsets[num] = BYTESWAP32(metaoffsets32[num]);
											} while (num > 0);

											free(metaoffsets32);
										}
									}
								}
								else
								{
									//size of null
									CloseSource((size_t)mp4);
									mp4 = NULL;
									break;
								}
							}
						}
						mp4->filepos += len;
						LongSeek(mp4, qtsize - 8 - len); // skip over stco
					}
					else
						LongSeek(mp4, qtsize - 8);

					NESTSIZE(qtsize);
				}

				else if (qttag == MAKEID('c', 'o', '6', '4')) // metadata stco - offsets
				{
					if (type == traktype) // meta
					{
						len = fread(&skip, 1, 4, mp4->mediafp);
						len += fread(&num, 1, 4, mp4->mediafp);
						num = BYTESWAP32(num);

						if(num == 0)
						{
							//size of null
							CloseSource((size_t)mp4);
							mp4 = NULL;
							break;
						}

						if (num <= (qtsize/sizeof(uint64_t)))
						{
							mp4->metastco_count = num;

							if (mp4->metastsc_count > 0 && num != mp4->metasize_count)
							{
								if (mp4->metaoffsets)
								{
									free(mp4->metaoffsets);
									mp4->metaoffsets = 0;
								}
								if (mp4->metasize_count && mp4->metasizes && qtsize > (num*8))
								{
									mp4->metaoffsets = (uint64_t *)malloc(mp4->metasize_count * 8);
									if (mp4->metaoffsets)
									{
										uint64_t *metaoffsets64 = NULL;
										metaoffsets64 = (uint64_t *)malloc(num * 8);
										if (metaoffsets64)
										{
											uint64_t fileoffset = 0;
											int stsc_pos = 0;
											int stco_pos = 0;
											len += fread(metaoffsets64, 1, num * 8, mp4->mediafp);
											do
											{
												num--;
												metaoffsets64[num] = BYTESWAP64(metaoffsets64[num]);
											} while (num > 0);

											fileoffset = metaoffsets64[0];
											mp4->metaoffsets[0] = fileoffset;
											//printf("%3d:%08x, delta = %08x\n", 0, (int)fileoffset, 0);

											num = 1;
											while (num < mp4->metasize_count)
											{
												if (num != mp4->metastsc[stsc_pos].chunk_num - 1 && mp4->metastsc[stsc_pos].samples && 0 == (num - (mp4->metastsc[stsc_pos].chunk_num - 1)) % mp4->metastsc[stsc_pos].samples)
												{
													stco_pos++;
													if(stco_pos < (int)mp4->metastco_count)
														fileoffset = metaoffsets64[stco_pos];
												}
												else
												{
													if(num <= mp4->indexcount)
														fileoffset += (uint64_t)mp4->metasizes[num - 1];
												}

												mp4->metaoffsets[num] = fileoffset;
												//int delta = metaoffsets[num] - metaoffsets[num - 1];
												//printf("%3d:%08x, delta = %08x\n", num, (int)fileoffset, delta);

												num++;
											}

											if (mp4->metastsc) free(mp4->metastsc);
											mp4->metastsc = NULL;
											mp4->metastsc_count = 0;

											free(metaoffsets64);
										}
									}
								}
								else
								{
									//size of null
									CloseSource((size_t)mp4);
									mp4 = NULL;
									break;
								}
							}
							else
							{
								if (mp4->metaoffsets)
								{
									free(mp4->metaoffsets);
									mp4->metaoffsets = 0;
								}
								if (qtsize > (num * 8))
								{
									mp4->metaoffsets = (uint64_t*)malloc(num * 8);
									if (mp4->metaoffsets)
									{
										len += fread(mp4->metaoffsets, 1, num * 8, mp4->mediafp);
										do
										{
											num--;
											mp4->metaoffsets[num] = BYTESWAP64(mp4->metaoffsets[num]);
										} while (num > 0);
									}
								}
							}
						}
						mp4->filepos += len;
						LongSeek(mp4, qtsize - 8 - len); // skip over stco
					}
					else
						LongSeek(mp4, qtsize - 8);

					NESTSIZE(qtsize);
				}
				else if (qttag == MAKEID('s', 't', 't', 's')) // time to samples
				{
					if (type == MAKEID('v', 'i', 'd', 'e')) // video trak to get frame rate
					{
						mp4->videolength = (float)((double)mp4->trak_clockcount / (double)mp4->trak_clockdemon);

						uint32_t samples = 0;
						uint32_t entries = 0;
						len = fread(&skip, 1, 4, mp4->mediafp);
						len += fread(&num, 1, 4, mp4->mediafp);
						num = BYTESWAP32(num);

						if(PRINT_MP4_STRUCTURE){
							printf("Video Time to sample has %d enteries.\n", num);
							printf("Sample Count \t Sample Duration \n");
						}
						if (num <= (qtsize / 8))
						{
							entries = num;

							while (entries > 0)
							{
								uint32_t samplecount;
								uint32_t duration;
								len += fread(&samplecount, 1, 4, mp4->mediafp);
								samplecount = BYTESWAP32(samplecount);
								len += fread(&duration, 1, 4, mp4->mediafp);
								duration = BYTESWAP32(duration);

								printf("%d  \t\t  %d\n", samplecount, duration);
								samples += samplecount;
								entries--;

								if (mp4->video_framerate_numerator == 0)
								{
									mp4->video_framerate_numerator = mp4->trak_clockdemon;
									mp4->video_framerate_denominator = duration;
								}
							}
							mp4->video_frames = samples;
						}
						mp4->filepos += len;
						LongSeek(mp4, qtsize - 8 - len); // skip over stco
					}
					else if (type == traktype) // meta
					{
						uint32_t totaldur = 0, samples = 0;
						uint32_t entries = 0;
						len = fread(&skip, 1, 4, mp4->mediafp);
						len += fread(&num, 1, 4, mp4->mediafp);
						num = BYTESWAP32(num);

						if(PRINT_MP4_STRUCTURE) {
							printf("Meta time to sample has %d enteries.\n", num);
							printf("Sample Count \t Sample Duration \n");
						}

						if (num <= (qtsize / 8))
						{
							entries = num;

							mp4->meta_clockdemon = mp4->trak_clockdemon;
							mp4->meta_clockcount = mp4->trak_clockcount;


							if(mp4->meta_clockdemon == 0)
							{
								//prevent divide by zero
								CloseSource((size_t)mp4);
								mp4 = NULL;
								break;
							}

							if(entries > 1){
								fprintf(stderr, "samples with different times. this case is not handled until now.");
								exit(1);
							}

							while (entries > 0)
							{
								uint32_t samplecount;
								uint32_t duration;
								len += fread(&samplecount, 1, 4, mp4->mediafp);
								samplecount = BYTESWAP32(samplecount);
								len += fread(&duration, 1, 4, mp4->mediafp);
								duration = BYTESWAP32(duration);

								samples += samplecount;
								entries--;
//								printf("enteries: %d", entries);

								printf("%d  \t\t  %d\n", samplecount, duration);

								totaldur += duration;
								mp4->metadatalength += (double)((double)samplecount * (double)duration / (double)mp4->meta_clockdemon);
								if (samplecount > 1 || entries == 1)
									mp4->basemetadataduration = mp4->metadatalength * (double)mp4->meta_clockdemon / (double)samples;
								}
							}
							mp4->filepos += len;
							LongSeek(mp4, qtsize - 8 - len); // skip over stco
						}
						else
							LongSeek(mp4, qtsize - 8);

					NESTSIZE(qtsize);
				}
				else
				{
					NESTSIZE(8);
				}
			}
			else
			{
				break;
			}
		} while (len > 0);

		if (mp4)
		{
			if (mp4->metasizes == NULL || mp4->metaoffsets == NULL)
			{
				CloseSource((size_t)mp4);
				mp4 = NULL;
			}
			
			// set the numbers of payload with both size and offset
			if (mp4 != NULL)
			{
				mp4->indexcount = mp4->metasize_count;
				if (mp4->metastco_count < mp4->indexcount)
					mp4->indexcount = mp4->metastco_count;
			}
		}
	}
	else
	{
		//	printf("Could not open %s for input\n", filename);
		//	exit(1);

		free(mp4);
		mp4 = NULL;
	}

	return (size_t)mp4;
}


float GetDuration(size_t handle)
{
	mp4object *mp4 = (mp4object *)handle;
	if (mp4 == NULL) return 0.0;

	return (float)mp4->metadatalength;
}

uint32_t GetVideoFrameRateAndCount(size_t handle, uint32_t *numer, uint32_t *demon)
{
	mp4object *mp4 = (mp4object *)handle;
	if (mp4 == NULL) return 0;

	if (numer != NULL && demon != NULL && mp4->video_frames > 0)
	{
		*numer = mp4->video_framerate_numerator;
		*demon = mp4->video_framerate_denominator;
		return mp4->video_frames;
	}
	return 0;
}

void CloseSource(size_t handle)
{
	mp4object *mp4 = (mp4object *)handle;
	if (mp4 == NULL) 
	{
		return;
	}

	if (mp4->mediafp)
	{
		fclose(mp4->mediafp);
		mp4->mediafp = NULL;
	}
	if (mp4->metasizes)
	{
		free(mp4->metasizes);
		mp4->metasizes = 0;
	}
	if (mp4->metaoffsets)
	{
		free(mp4->metaoffsets);
		mp4->metaoffsets = 0;
	}
	if (mp4->metastsc)
	{
		free(mp4->metastsc);
		mp4->metastsc = 0;
	}
 
 	free(mp4);
}


uint32_t GetPayloadTime(size_t handle, uint32_t index, double *in, double *out)
{
	mp4object *mp4 = (mp4object *)handle;
	if (mp4 == NULL) return GPMF_ERROR_MEMORY;

	if (mp4->metaoffsets == 0 || mp4->basemetadataduration == 0 || mp4->meta_clockdemon == 0 || in == NULL || out == NULL) return GPMF_ERROR_MEMORY;

	*in = ((double)index * (double)mp4->basemetadataduration / (double)mp4->meta_clockdemon);
	*out = ((double)(index + 1) * (double)mp4->basemetadataduration / (double)mp4->meta_clockdemon);

	if (*out > (double)mp4->metadatalength)
		*out = (double)mp4->metadatalength;

	// Add any Edit List offset
	*in += (double)mp4->metadataoffset_clockcount / (double)mp4->clockdemon;
	*out += (double)mp4->metadataoffset_clockcount / (double)mp4->clockdemon;
	return GPMF_OK;
}


uint32_t GetPayloadRationalTime(size_t handle, uint32_t index, int32_t *in_numerator, int32_t *out_numerator, uint32_t *denominator)
{
    mp4object *mp4 = (mp4object *)handle;
    if (mp4 == NULL) return GPMF_ERROR_MEMORY;
    
    if (mp4->metaoffsets == 0 || mp4->basemetadataduration == 0 || mp4->meta_clockdemon == 0 || in_numerator == NULL || out_numerator == NULL) return GPMF_ERROR_MEMORY;

	*in_numerator = (int32_t)(index * mp4->basemetadataduration);
	*out_numerator = (int32_t)((index + 1) * mp4->basemetadataduration);

	if (*out_numerator > (int32_t)((double)mp4->metadatalength*(double)mp4->meta_clockdemon))
		*out_numerator = (int32_t)((double)mp4->metadatalength*(double)mp4->meta_clockdemon);

	// Add any Edit List offset
	*in_numerator += (int32_t)(((double)mp4->metadataoffset_clockcount / (double)mp4->clockdemon) * mp4->meta_clockdemon);
	*out_numerator += (int32_t)(((double)mp4->metadataoffset_clockcount / (double)mp4->clockdemon) * mp4->meta_clockdemon);

	*denominator = mp4->meta_clockdemon;
    
    return GPMF_OK;
}


uint32_t GetEditListOffset(size_t handle, double *offset)
{
	mp4object *mp4 = (mp4object *)handle;
	if (mp4 == NULL) return GPMF_ERROR_MEMORY;

	if (mp4->clockdemon == 0) return GPMF_ERROR_MEMORY;

	*offset = (double)mp4->metadataoffset_clockcount / (double)mp4->clockdemon;

	return GPMF_OK;
}

uint32_t GetEditListOffsetRationalTime(size_t handle, int32_t *offset_numerator, uint32_t *denominator)
{
	mp4object *mp4 = (mp4object *)handle;
	if (mp4 == NULL) return GPMF_ERROR_MEMORY;

	if (mp4->clockdemon == 0) return GPMF_ERROR_MEMORY;

	*offset_numerator = mp4->metadataoffset_clockcount;
	*denominator = mp4->clockdemon;

	return GPMF_OK;
}



size_t OpenMP4SourceUDTA(char *filename)
{
	mp4object *mp4 = (mp4object *)malloc(sizeof(mp4object));
	if (mp4 == NULL) return 0;

	memset(mp4, 0, sizeof(mp4object));

#ifdef _WINDOWS
	struct _stat64 mp4stat;
	_stat64(filename, &mp4stat);
#else
	struct stat mp4stat;
	stat(filename, &mp4stat);
#endif
	mp4->filesize = (uint64_t)mp4stat.st_size;
	if (mp4->filesize < 64) 
	{
		free(mp4);
		return 0;
	}

#ifdef _WINDOWS
	fopen_s(&mp4->mediafp, filename, "rb+");
#else
	mp4->mediafp = fopen(filename, "rb+");
#endif

	if (mp4->mediafp)
	{
		uint32_t qttag, qtsize32;
		size_t len;
		int32_t nest = 0;
		uint64_t nestsize[MAX_NEST_LEVEL] = { 0 };
		uint64_t lastsize = 0, qtsize;

		do
		{
			len = fread(&qtsize32, 1, 4, mp4->mediafp);
			len += fread(&qttag, 1, 4, mp4->mediafp);
			if (len == 8)
			{
				if (!GPMF_VALID_FOURCC(qttag))
				{
					mp4->filepos += len;
					LongSeek(mp4, lastsize - 8 - len);

					NESTSIZE(lastsize - 8);
					continue;
				}

				qtsize32 = BYTESWAP32(qtsize32);

				if (qtsize32 == 1) // 64-bit Atom
				{
					len += fread(&qtsize, 1, 8, mp4->mediafp);
					mp4->filepos += len;
					qtsize = BYTESWAP64(qtsize) - 8;
				}
				else
					qtsize = qtsize32;

				nest++;

				if (qtsize < 8) break;
				if (nest >= MAX_NEST_LEVEL) break;

				nestsize[nest] = qtsize;
				lastsize = qtsize;

				if (qttag == MAKEID('m', 'd', 'a', 't') ||
					qttag == MAKEID('f', 't', 'y', 'p'))
				{
					LongSeek(mp4, qtsize - 8);
					NESTSIZE(qtsize);
					continue;
				}

				if (qttag == MAKEID('G', 'P', 'M', 'F'))
				{
					mp4->videolength += 1.0;
					mp4->metadatalength += 1.0;

					mp4->indexcount = (uint32_t)mp4->metadatalength;

					mp4->metasizes = (uint32_t *)malloc(mp4->indexcount * 4 + 4);  memset(mp4->metasizes, 0, mp4->indexcount * 4 + 4);
					mp4->metaoffsets = (uint64_t *)malloc(mp4->indexcount * 8 + 8);  memset(mp4->metaoffsets, 0, mp4->indexcount * 8 + 8);

					mp4->metasizes[0] = (uint32_t)qtsize - 8;
					mp4->metaoffsets[0] = (uint64_t) LONGTELL(mp4->mediafp);
					mp4->metasize_count = 1;

					return (size_t)mp4;  // not an MP4, RAW GPMF which has not inherent timing, assigning a during of 1second.
				}
				if (qttag != MAKEID('m', 'o', 'o', 'v') && //skip over all but these atoms
					qttag != MAKEID('u', 'd', 't', 'a'))
				{
					LongSeek(mp4, qtsize - 8);
					NESTSIZE(qtsize);
					continue;
				}
				else
				{
					NESTSIZE(8);
				}
			}
		} while (len > 0);
	}
	return (size_t)mp4;
}


void SetTimeBaseStream(size_t handle, uint32_t fourcc)
{
	mp4object* mp4 = (mp4object*)handle;
	if (mp4 == NULL) return;

	mp4->timeBaseFourCC = 0;

	if (!GPMF_VALID_FOURCC(fourcc)) return;

	mp4->timeBaseFourCC = fourcc;
}


double GetGPMFSampleRate(size_t handle, uint32_t fourcc, uint32_t flags, double *firstsampletime, double *lastsampletime)
{
	mp4object *mp4 = (mp4object *)handle;
	if (mp4 == NULL) return 0.0;

	GPMF_stream metadata_stream, *ms = &metadata_stream;
	uint32_t teststart = 0;
	uint32_t testend = mp4->indexcount;
	double rate = 0.0;

	uint32_t *payload;
	uint32_t payloadsize;
	GPMF_ERR ret;

	if (mp4->indexcount < 1)
		return 0.0;

	payload = GetPayload(handle, NULL, teststart); 
	payloadsize = GetPayloadSize(handle, teststart);
	ret = GPMF_Init(ms, payload, payloadsize);

	if (ret != GPMF_OK)
		goto cleanup;

	{
		uint64_t basetimestamp = 0;
		uint64_t starttimestamp = 0;
		uint64_t endtimestamp = 0;
		uint32_t startsamples = 0;
		uint32_t endsamples = 0;
		double intercept = 0.0;



		while (teststart < mp4->indexcount && ret == GPMF_OK && GPMF_OK != GPMF_FindNext(ms, fourcc, GPMF_RECURSE_LEVELS))
		{
			teststart++;
			payload = GetPayload(handle, payload, teststart); // second last payload
			payloadsize = GetPayloadSize(handle, teststart);
			ret = GPMF_Init(ms, payload, payloadsize);
		}

		if (ret == GPMF_OK && payload)
		{
			double startin, startout, endin, endout;
			int usedTimeStamps = 0;

			uint32_t samples = GPMF_PayloadSampleCount(ms);
			GPMF_stream find_stream;
			GPMF_CopyState(ms, &find_stream);  //ms is at the searched fourcc
			if (GPMF_OK == GPMF_FindPrev(&find_stream, GPMF_KEY_TOTAL_SAMPLES, GPMF_CURRENT_LEVEL))
				startsamples = BYTESWAP32(*(uint32_t *)GPMF_RawData(&find_stream)) - samples;

			GPMF_CopyState(ms, &find_stream);
			if (GPMF_OK == GPMF_FindPrev(&find_stream, GPMF_KEY_TIME_STAMP, GPMF_CURRENT_LEVEL))
				starttimestamp = BYTESWAP64(*(uint64_t *)GPMF_RawData(&find_stream));

			if (starttimestamp) // how does this compare to other streams in this early payload?
			{
				GPMF_stream any_stream;
				if (GPMF_OK == GPMF_Init(&any_stream, payload, payloadsize))
				{
					basetimestamp = starttimestamp;  

					if (mp4->timeBaseFourCC)
					{
						if (GPMF_OK == GPMF_FindNext(&any_stream, mp4->timeBaseFourCC, GPMF_RECURSE_LEVELS))
						{
							if (GPMF_OK == GPMF_FindPrev(&any_stream, GPMF_KEY_TIME_STAMP, GPMF_CURRENT_LEVEL))
							{
								basetimestamp = BYTESWAP64(*(uint64_t*)GPMF_RawData(&any_stream));
							}
						}

					}
					else
					{
						while (GPMF_OK == GPMF_FindNext(&any_stream, GPMF_KEY_TIME_STAMP, GPMF_RECURSE_LEVELS))
						{
							uint64_t timestamp = BYTESWAP64(*(uint64_t*)GPMF_RawData(&any_stream));
							if (timestamp < basetimestamp)
								basetimestamp = timestamp;
						}
					}
				}
			}
			//Note: basetimestamp is used the remove offset from the timestamp, 
			// however 0.0 may not be the same zero for your video or audio presentation time (although it should be close.)
			// On GoPro camera, metadata streams like SHUT and ISOE are metadata fields associated with video, and these can be used
			// to accurately sync meta with video.

			testend = mp4->indexcount;
			do
			{
				testend--;// last payload with the fourcc needed
				payload = GetPayload(handle, payload, testend);
				payloadsize = GetPayloadSize(handle, testend);
				ret = GPMF_Init(ms, payload, payloadsize);
			} while (testend > 0 && ret == GPMF_OK &&  GPMF_OK != GPMF_FindNext(ms, fourcc, GPMF_RECURSE_LEVELS));

			GetPayloadTime(handle, teststart, &startin, &startout);
			GetPayloadTime(handle, testend, &endin, &endout);

			GPMF_CopyState(ms, &find_stream);
			if (GPMF_OK == GPMF_FindPrev(&find_stream, GPMF_KEY_TOTAL_SAMPLES, GPMF_CURRENT_LEVEL))
				endsamples = BYTESWAP32(*(uint32_t *)GPMF_RawData(&find_stream));
			else // If there is no TSMP we have to count the samples.
			{
				uint32_t i;
				for (i = teststart; i <= testend; i++)
				{
					payload = GetPayload(handle,payload, i); // second last payload
					payloadsize = GetPayloadSize(handle, i);
					if (GPMF_OK == GPMF_Init(ms, payload, payloadsize))
						if (GPMF_OK == GPMF_FindNext(ms, fourcc, GPMF_RECURSE_LEVELS))
							endsamples += GPMF_PayloadSampleCount(ms);
				}
			}

			if (starttimestamp != 0)
			{
				uint32_t last_samples = GPMF_PayloadSampleCount(ms);
				uint32_t totaltimestamped_samples = endsamples - last_samples - startsamples;
				double time_stamp_scale = 1000000000.0; // scan for nanoseconds, microseconds to seconds, all base 10.

				GPMF_CopyState(ms, &find_stream);
				if (GPMF_OK == GPMF_FindPrev(&find_stream, GPMF_KEY_TIME_STAMP, GPMF_CURRENT_LEVEL))
					endtimestamp = BYTESWAP64(*(uint64_t *)GPMF_RawData(&find_stream));

				if (endtimestamp)
				{
					double approxrate = 0.0;
					if (endsamples > startsamples)
						approxrate = (double)(endsamples - startsamples) / (endout - startin);

					if (approxrate == 0.0)
						approxrate = (double)(samples) / (endout - startin);


					while (time_stamp_scale >= 1)
					{
						rate = (double)(totaltimestamped_samples) / ((double)(endtimestamp - starttimestamp) / time_stamp_scale);
						if (rate*0.9 < approxrate && approxrate < rate*1.1)
							break;

						time_stamp_scale *= 0.1;
					}
					if (time_stamp_scale < 1.0) rate = 0.0;
					intercept = (((double)basetimestamp - (double)starttimestamp) / time_stamp_scale) * rate;
					usedTimeStamps = 1;
				}
			}

			if (rate == 0.0) //Timestamps didn't help, or weren't available
			{
				if (!(flags & GPMF_SAMPLE_RATE_PRECISE))
				{
					if (endsamples > startsamples)
						rate = (double)(endsamples - startsamples) / (endout - startin);

					if (rate == 0.0)
						rate = (double)(samples) / (endout - startin);

					intercept = (double)-startin * rate;
				}
				else // for increased precision, for older GPMF streams sometimes missing the total sample count 
				{
					uint32_t payloadpos = 0, payloadcount = 0;
					double slope, top = 0.0, bot = 0.0, meanX = 0, meanY = 0;
					uint32_t *repeatarray = (uint32_t *)malloc(mp4->indexcount * 4 + 4);
					memset(repeatarray, 0, mp4->indexcount * 4 + 4);

					samples = 0;

					for (payloadpos = teststart; payloadpos <= testend; payloadpos++)
					{
						payload = GetPayload(handle, payload, payloadpos); // second last payload
						payloadsize = GetPayloadSize(handle, payloadpos);
						ret = GPMF_Init(ms, payload, payloadsize);

						if (ret != GPMF_OK)
							goto cleanup;

						if (GPMF_OK == GPMF_FindNext(ms, fourcc, GPMF_RECURSE_LEVELS))
						{
							GPMF_stream find_stream2;
							GPMF_CopyState(ms, &find_stream2);

							payloadcount++;

							if (GPMF_OK == GPMF_FindNext(&find_stream2, fourcc, GPMF_CURRENT_LEVEL)) // Count the instances, not the repeats
							{
								if (repeatarray)
								{
									double in, out;

									do
									{
										samples++;
									} while (GPMF_OK == GPMF_FindNext(ms, fourcc, GPMF_CURRENT_LEVEL));

									repeatarray[payloadpos] = samples;
									meanY += (double)samples;

									if (GPMF_OK == GetPayloadTime(handle, payloadpos, &in, &out))
										meanX += out;
								}
							}
							else
							{
								uint32_t repeat = GPMF_PayloadSampleCount(ms);
								samples += repeat;

								if (repeatarray)
								{
									double in, out;

									repeatarray[payloadpos] = samples;
									meanY += (double)samples;

									if (GPMF_OK == GetPayloadTime(handle, payloadpos, &in, &out))
										meanX += out;
								}
							}
						}
						else
						{
							repeatarray[payloadpos] = 0;
						}
					}

					// Compute the line of best fit for a jitter removed sample rate.  
					// This does assume an unchanging clock, even though the IMU data can thermally impacted causing small clock changes.  
					// TODO: Next enhancement would be a low order polynominal fit the compensate for any thermal clock drift.
					if (repeatarray)
					{
						meanY /= (double)payloadcount;
						meanX /= (double)payloadcount;

						for (payloadpos = teststart; payloadpos <= testend; payloadpos++)
						{
							double in, out;
							if (repeatarray[payloadpos] && GPMF_OK == GetPayloadTime(handle, payloadpos, &in, &out))
							{
								top += ((double)out - meanX)*((double)repeatarray[payloadpos] - meanY);
								bot += ((double)out - meanX)*((double)out - meanX);
							}
						}

						slope = top / bot;
						rate = slope;

						// This sample code might be useful for compare data latency between channels.
						intercept = meanY - slope * meanX;
#if 0
						printf("%c%c%c%c start offset = %f (%.3fms) rate = %f\n", PRINTF_4CC(fourcc), intercept, 1000.0 * intercept / slope, rate);
						printf("%c%c%c%c first sample at time %.3fms\n", PRINTF_4CC(fourcc), -1000.0 * intercept / slope);
#endif
					}
					else
					{
						rate = (double)(samples) / (endout - startin);
					}

					free(repeatarray);
				}
			}

			if (firstsampletime && lastsampletime)
			{
				uint32_t endpayload = mp4->indexcount;
				do
				{
					endpayload--;// last payload with the fourcc needed
					payload = GetPayload(handle, payload, endpayload);
					payloadsize = GetPayloadSize(handle, endpayload);
					ret = GPMF_Init(ms, payload, payloadsize);
				} while (endpayload > 0 && ret == GPMF_OK && GPMF_OK != GPMF_FindNext(ms, fourcc, GPMF_RECURSE_LEVELS));

				if (endpayload > 0 && ret == GPMF_OK)
				{
					uint32_t totalsamples = endsamples - startsamples;
					float timo = 0.0;

					GPMF_CopyState(ms, &find_stream);
					if (GPMF_OK == GPMF_FindPrev(&find_stream, GPMF_KEY_TIME_OFFSET, GPMF_CURRENT_LEVEL))
						GPMF_FormattedData(&find_stream, &timo, 4, 0, 1);

					double first, last;
					first = -intercept / rate - timo;
					last = first + (double)totalsamples / rate;

					//Apply any Edit List corrections.
					if (usedTimeStamps)  // clips with STMP have the Edit List already applied via GetPayloadTime()
					{
						first += (double)mp4->metadataoffset_clockcount / (double)mp4->clockdemon;
						last += (double)mp4->metadataoffset_clockcount / (double)mp4->clockdemon;
					}

					//printf("%c%c%c%c first sample at time %.3fms, last at %.3fms\n", PRINTF_4CC(fourcc), 1000.0*first, 1000.0*last);

					if (firstsampletime) *firstsampletime = first;

					if (lastsampletime) *lastsampletime = last;
				}
			}
		}
	}

cleanup:
	if (payload)
		FreePayload(payload);
	payload = NULL;

	return rate;
}

