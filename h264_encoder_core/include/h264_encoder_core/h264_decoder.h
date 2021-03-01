/*
 *  Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#pragma once

#include <stdint.h>

#include <memory>
#include <vector>

extern "C" {
#include <libavutil/pixfmt.h>
}

#include <aws_common/sdk_utils/parameter_reader.h>

namespace Aws {
namespace Utils {
namespace Decoding {

struct H264DecoderInput
{
  H264DecoderInput(const std::vector<uint8_t>& data) : frame_data(data) { Reset(); }

  void Reset()
  {
    frame_pts = 0;
    frame_dts = 0;
    frame_duration = 0;
    key_frame = false;
  }

  const std::vector<uint8_t>& frame_data;
  uint64_t frame_pts;
  uint64_t frame_dts;
  uint64_t frame_duration;
  bool key_frame;
};

struct H264DecoderOutput
{
  H264DecoderOutput(std::vector<uint8_t>& data) : frame_data(data) { Reset(); }

  void Reset()
  {
    frame_data.clear();
    height = 0;
    width = 0;
    stride = 0;
    encoding = AV_PIX_FMT_NONE;
  }

  std::vector<uint8_t>& frame_data;
  uint64_t height;
  uint64_t width;
  uint64_t stride;
  AVPixelFormat encoding;
};

class H264DecoderImpl;

class H264Decoder
{
public:
  H264Decoder();

  ~H264Decoder();

  /**
   * Initialize the H264Encoder instance
   * @param src_width the width of the source video stream
   * @param src_height the height of the source video stream
   * @param src_encoding the encoding of the source video stream
   * @param dst_params parameter reader used for reading the desired configuration of the encoder
   * output
   * @return AWS_ERR_OK if initialization was successful
   */
  AwsError Initialize(const AVPixelFormat dst_encoding, const Aws::Client::ParameterReaderInterface & dst_params);

  /**
   * Encode one frame
   * @param img_data frame data
   * @return resulting struct containing H264 encoded data
   */
  // AwsError Encode(const uint8_t * img_data, H264EncoderResult & res) const;
  AwsError Decode(const H264DecoderInput & img_data, H264DecoderOutput& res) const;

private:
  std::unique_ptr<H264DecoderImpl> impl_;
};

}  // namespace Encoding
}  // namespace Utils
}  // namespace Aws
