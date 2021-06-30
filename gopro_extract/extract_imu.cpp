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

#include "extract_imu.h"
#include "utils.h"
#include <fstream>
#include <iomanip>
#include <cstdlib>

extern void PrintGPMF(GPMF_stream *ms);

using namespace std;

GoProIMUExtractor::GoProIMUExtractor(const std::string file)
{
    strcpy(video, file.c_str());
    ms = &metadata_stream;
    mp4 = OpenMP4Source(video, MOV_GPMF_TRAK_TYPE, MOV_GPMF_TRAK_SUBTYPE, 0);
    if (mp4 == 0)
    {
        std::cout << RED << "Could not open video file" << RESET << std::endl;
    }
    metadatalength = GetDuration(mp4);

    if (metadatalength > 0.0)
    {
        payloads = GetNumberPayloads(mp4);
    }
}

bool GoProIMUExtractor::display_video_framerate()
{
    uint32_t fr_num, fr_dem;
    uint32_t frames = GetVideoFrameRateAndCount(mp4, &fr_num, &fr_dem);
    if (frames)
    {
        printf("VIDEO FRAMERATE:\n  %.3f with %d frames\n", (float)fr_num / (float)fr_dem, frames);
        return true;
    }
    else
    {
        return false;
    }
}

void GoProIMUExtractor::cleanup()
{
    if (payloadres)
        FreePayloadResource(mp4, payloadres);
    if (ms)
        GPMF_Free(ms);

    payload = NULL;
    CloseSource(mp4);
}

void GoProIMUExtractor::show_gpmf_structure()
{
    uint32_t payload_size;
    GPMF_ERR ret = GPMF_OK;

    // Just print the structure of first payload
    // Remaining structure should also be similar
    uint32_t index = 0;
    double in = 0.0, out = 0.0; //times

    payload_size = GetPayloadSize(mp4, index);
    payloadres = GetPayloadResource(mp4, payloadres, payload_size);
    payload = GetPayload(mp4, payloadres, index);

    if (payload == NULL)
        cleanup();

    ret = GetPayloadTime(mp4, index, &in, &out);
    if (ret != GPMF_OK)
        cleanup();

    ret = GPMF_Init(ms, payload, payload_size);
    if (ret != GPMF_OK)
        cleanup();

    printf("PAYLOAD TIME:\n  %.3f to %.3f seconds\n", in, out);
    printf("GPMF STRUCTURE:\n");
    // Output (printf) all the contained GPMF data within this payload
    ret = GPMF_Validate(ms, GPMF_RECURSE_LEVELS); // optional
    if (GPMF_OK != ret)
    {
        if (GPMF_ERROR_UNKNOWN_TYPE == ret)
        {
            printf("Unknown GPMF Type within, ignoring\n");
            ret = GPMF_OK;
        }
        else
            printf("Invalid GPMF Structure\n");
    }

    GPMF_ResetState(ms);

    GPMF_ERR nextret;
    do
    {
        printf("  ");
        PrintGPMF(ms); // printf current GPMF KLV

        nextret = GPMF_Next(ms, GPMF_RECURSE_LEVELS);

        while (nextret == GPMF_ERROR_UNKNOWN_TYPE) // or just using GPMF_Next(ms, GPMF_RECURSE_LEVELS|GPMF_TOLERANT) to ignore and skip unknown types
            nextret = GPMF_Next(ms, GPMF_RECURSE_LEVELS);

    } while (GPMF_OK == nextret);
    GPMF_ResetState(ms);
}

/** Only supports native formats like ACCL, GYRO, GPS
 *
 * @param fourcc
 * @param readings
 * @param timestamp
 * @return
 */

GPMF_ERR GoProIMUExtractor::get_scaled_data(uint32_t fourcc, vector<vector<double>> &readings)
{

    while (GPMF_OK == GPMF_FindNext(ms, STR2FOURCC("STRM"), static_cast<GPMF_LEVELS>(GPMF_RECURSE_LEVELS | GPMF_TOLERANT))) //GoPro Hero5/6/7 Accelerometer)
    {
        if (GPMF_OK != GPMF_FindNext(ms, fourcc, static_cast<GPMF_LEVELS>(GPMF_RECURSE_LEVELS | GPMF_TOLERANT)))
            continue;

        uint32_t samples = GPMF_Repeat(ms);
        uint32_t elements = GPMF_ElementsInStruct(ms);
        uint32_t buffersize = samples * elements * sizeof(double);
        double *ptr, *tmpbuffer = (double *)malloc(buffersize);

        readings.resize(samples);

        if (tmpbuffer && samples)
        {
            uint32_t i, j;

            //GPMF_FormattedData(ms, tmpbuffer, buffersize, 0, samples); // Output data in LittleEnd, but no scale
            if (GPMF_OK == GPMF_ScaledData(ms, tmpbuffer, buffersize, 0, samples, GPMF_TYPE_DOUBLE)) //Output scaled data as floats
            {

                ptr = tmpbuffer;
                for (i = 0; i < samples; i++)
                {
                    vector<double> sample(elements);
                    for (j = 0; j < elements; j++)
                    {
                        sample.at(j) = *ptr++;
                    }
                    readings.at(i) = sample;
                }
            }
            free(tmpbuffer);
        }
    }

    GPMF_ResetState(ms);
    return GPMF_OK;
}

/** returns STMP (start time of current payload)
 *
 * @param fourcc
 * @return timestmamp
 */

uint64_t GoProIMUExtractor::get_stamp(uint32_t fourcc)
{
    GPMF_stream find_stream;

    uint64_t timestamp;
    while (GPMF_OK == GPMF_FindNext(ms, STR2FOURCC("STRM"), static_cast<GPMF_LEVELS>(GPMF_RECURSE_LEVELS | GPMF_TOLERANT))) //GoPro Hero5/6/7 Accelerometer)
    {
        if (GPMF_OK != GPMF_FindNext(ms, fourcc, static_cast<GPMF_LEVELS>(GPMF_RECURSE_LEVELS | GPMF_TOLERANT)))
            continue;

        GPMF_CopyState(ms, &find_stream);
        if (GPMF_OK == GPMF_FindPrev(&find_stream, GPMF_KEY_TIME_STAMP,
                                     static_cast<GPMF_LEVELS>(GPMF_CURRENT_LEVEL | GPMF_TOLERANT)))
            timestamp = BYTESWAP64(*(uint64_t *)GPMF_RawData(&find_stream));
    }
    GPMF_ResetState(ms);
    return timestamp;
}

GPMF_ERR GoProIMUExtractor::show_current_payload(uint32_t index)
{

    uint32_t payload_size;
    GPMF_ERR ret;

    payload_size = GetPayloadSize(mp4, index);
    payloadres = GetPayloadResource(mp4, payloadres, payload_size);
    payload = GetPayload(mp4, payloadres, index);

    if (payload == NULL)
        cleanup();
    ret = GPMF_Init(ms, payload, payload_size);
    if (ret != GPMF_OK)
        cleanup();

    GPMF_ERR nextret;
    do
    {
        printf("  ");
        PrintGPMF(ms); // printf current GPMF KLV

        nextret = GPMF_Next(ms, GPMF_RECURSE_LEVELS);

        while (nextret == GPMF_ERROR_UNKNOWN_TYPE) // or just using GPMF_Next(ms, GPMF_RECURSE_LEVELS|GPMF_TOLERANT) to ignore and skip unknown types
            nextret = GPMF_Next(ms, GPMF_RECURSE_LEVELS);

    } while (GPMF_OK == nextret);
    GPMF_ResetState(ms);
}

int GoProIMUExtractor::save_imu_stream(std::string imu_file)
{

    ofstream imu_stream;
    imu_stream.open(imu_file);
    imu_stream << std::fixed << std::setprecision(19);
    imu_stream << "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],"
                  "a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]"
               << endl;

    uint32_t index;
    mp4object *mp4_obj = (mp4object *)mp4;
    uint32_t movie_creation_time = mp4_obj->movie_creation_time;
    uint64_t unix_creation_time = (uint64_t)movie_creation_time - get_offset_1904();
    unix_creation_time = unix_creation_time * 1000000000;

    uint64_t first_frame_us, first_frame_ns;
    vector<vector<double>> accl_data;
    vector<vector<double>> gyro_data;

    uint64_t current_stamp, prev_stamp;

    vector<uint64_t> steps;
    uint64_t total_samples = 0;

    for (index = 0; index < payloads; index++)
    {
        GPMF_ERR ret;
        uint32_t payload_size;

        payload_size = GetPayloadSize(mp4, index);
        payloadres = GetPayloadResource(mp4, payloadres, payload_size);
        payload = GetPayload(mp4, payloadres, index);

        if (payload == NULL)
            cleanup();
        ret = GPMF_Init(ms, payload, payload_size);
        if (ret != GPMF_OK)
            cleanup();

        if (index == 0)
        {
            first_frame_us = get_stamp(STR2FOURCC("CORI"));
            first_frame_ns = first_frame_us * 1000;
        }

        current_stamp = get_stamp(STR2FOURCC("ACCL"));
        uint64_t current_gyro_stamp = get_stamp(STR2FOURCC("GYRO"));

        if (current_gyro_stamp != current_stamp)
        {
            int32_t diff = current_gyro_stamp - current_stamp;
            if (abs(diff) > 20)
            {
                cout << RED << "[ERROR] ACCL and GYRO timestamp heavily un-synchronized ....Shutting Down!!!" << RESET << endl;
                std::cout << RED << "Index: " << index << " accl stamp: " << current_stamp << " gyro stamp: " << current_gyro_stamp << RESET << endl;
                exit(1);
            }
            else
            {
                cout << YELLOW << "[WARN] ACCL and GYRO timestamp slightly not synchronized ...." << RESET << endl;
                std::cout << YELLOW << "Index: " << index << " accl stamp: " << current_stamp << " gyro stamp: " << current_gyro_stamp << RESET << endl;
            }

            //            show_current_payload(index);
        }

        current_stamp *= 1000;
        if (index > 0)
        {
            uint64_t time_span = current_stamp - prev_stamp;
            if (time_span < 0)
            {
                cout << RED << "previous timestamp should be smaller than current stamp" << RESET << endl;
                exit(1);
            }

            uint64_t step_size = time_span / accl_data.size();
            steps.emplace_back(step_size);

            if (accl_data.size() != gyro_data.size())
            {
                cout << RED << "ACCL and GYRO data must be of same size" << RESET << endl;
                exit(1);
            }

            for (int i = 0; i < gyro_data.size(); ++i)
            {
                uint64_t s = prev_stamp + i * step_size;
                uint64_t ros_stamp = unix_creation_time + s - first_frame_ns;
                imu_stream << uint64_to_string(ros_stamp);

                vector<double> gyro_sample = gyro_data.at(i);

                // Data comes in ZXY order
                imu_stream << "," << gyro_sample.at(1);
                imu_stream << "," << gyro_sample.at(2);
                imu_stream << "," << gyro_sample.at(0);

                vector<double> accl_sample = accl_data.at(i);
                imu_stream << "," << accl_sample.at(1);
                imu_stream << "," << accl_sample.at(2);
                imu_stream << "," << accl_sample.at(0) << endl;
            }
        }

        gyro_data.clear();
        accl_data.clear();
        get_scaled_data(STR2FOURCC("ACCL"), accl_data);
        get_scaled_data(STR2FOURCC("GYRO"), gyro_data);

        total_samples += gyro_data.size();
        //        double start_stamp_secs = ((double) current_stamp)*1e-9;
        //        std::cout << "Payload Start Stamp: " << start_stamp_secs << "\tSamples: " << accl_data.size() << endl;
        prev_stamp = current_stamp;

        GPMF_Free(ms);
    }

    uint64_t mean_step_size = 0;
    for (double step_size : steps)
    {
        mean_step_size += step_size;
    }
    mean_step_size /= steps.size();

    for (int i = 0; i < gyro_data.size(); ++i)
    {
        uint64_t s = prev_stamp + i * mean_step_size;
        uint64_t ros_stamp = unix_creation_time + s - first_frame_ns;
        imu_stream << uint64_to_string(ros_stamp);

        vector<double> gyro_sample = gyro_data.at(i);

        // Data comes in ZXY order
        imu_stream << "," << gyro_sample.at(1);
        imu_stream << "," << gyro_sample.at(2);
        imu_stream << "," << gyro_sample.at(0);

        vector<double> accl_sample = accl_data.at(i);
        imu_stream << "," << accl_sample.at(1);
        imu_stream << "," << accl_sample.at(2);
        imu_stream << "," << accl_sample.at(0) << endl;
    }

    std::cout << GREEN << "Wrote " << total_samples << " imu samples to file" << RESET << endl;
    imu_stream.close();
    CloseSource(mp4);
}
