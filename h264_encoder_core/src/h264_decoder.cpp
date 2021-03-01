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

#include <aws/core/utils/logging/LogMacros.h>
#include <h264_encoder_core/h264_decoder.h>

#include <cstdio>
#include <dlfcn.h>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}

using namespace Aws::Client;
using namespace Aws::Utils::Logging;



namespace Aws {
namespace Utils {
namespace Decoding {

constexpr char kOutputWidthKey[] = "output_width";
constexpr char kOutputHeightKey[] = "output_height";
constexpr char kFpsNumeratorKey[] = "fps_numerator";
constexpr char kFpsDenominatorKey[] = "fps_denominator";
constexpr char kCodecKey[] = "codec";
constexpr char kBitrateKey[] = "bitrate";

constexpr char kDefaultHardwareCodec[] = "h264";
constexpr char kDefaultSoftwareCodec[] = "libx264";
constexpr float kFragmentDuration = 1.0f; /* set fragment duration to 1.0 second */
constexpr int kDefaultMaxBFrames = 0;
constexpr int kDefaultFpsNumerator = 24;
constexpr int kDefaultFpsDenominator = 1;
constexpr int kDefaultBitrate = 2048000;

class H264DecoderImpl
{
public:
  H264DecoderImpl()
  : width_(-1),
    height_(-1),
    dst_encoding_(AV_PIX_FMT_NONE),
    dst_stride_(-1),
    convert_ctx_(nullptr),
    param_(nullptr)
  {
  }

  /* Setup param_ 
   * Function will fail if param_ is not nullptr
   */
  AwsError set_param(AVCodec * codec) {
    if (nullptr != param_) {
      AWS_LOG_ERROR(__func__, "Unable to setup codec context. param_ must be null");
      return AWS_ERR_FAILURE;
    }
    param_ = avcodec_alloc_context3(codec);
    if (nullptr == param_) {
      AWS_LOG_ERROR(__func__, "Could not allocate video codec context");
      return AWS_ERR_MEMORY;
    }
    // /* put sample parameters */
    // param_->bit_rate = bitrate_;
    // /* resolution must be a multiple of two */
    // param_->width = dst_width_;
    // param_->height = dst_height_;
    /* frames per second */
    // param_->time_base = (AVRational){fps_den_, fps_num_};
    // frame_duration_ = (1e6 * fps_den_) / fps_num_;
    // param_->gop_size = static_cast<int>(ceil(kFragmentDuration * fps_num_ / fps_den_));
    // param_->keyint_min = param_->gop_size - 1;
    // param_->max_b_frames = kDefaultMaxBFrames;
    // param_->pix_fmt = AV_PIX_FMT_YUV420P;

    // param_->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
    // param_->flags2 &= ~AV_CODEC_FLAG2_LOCAL_HEADER;

    return AWS_ERR_OK;
  }
  
  // Attempts to open a codec
  AwsError open_codec(AVCodec * codec, AVDictionary * opts) {
    if (nullptr == codec) {
      AWS_LOG_ERROR(__func__, "Invalid codec");
      return AWS_ERR_FAILURE;
    }

    AWS_LOGSTREAM_INFO(__func__, "Attempting to open codec: " << codec->name);

    if (AWS_ERR_OK != set_param(codec) || avcodec_open2(param_, codec, &opts) < 0 ) {
      AWS_LOG_ERROR(__func__, "Could not open codec");
      if (nullptr != param_) {
	      avcodec_close(param_);
	      av_free(param_);
	      param_ = nullptr;
      }
      return AWS_ERR_FAILURE;
    }
   
    return AWS_ERR_OK;
  }

  AwsError Initialize(const std::string & codec_name)
  {
    // if (src_width <= 0) {
    //   AWS_LOGSTREAM_ERROR(__func__, "Invalid video source width " << src_width << "!");
    //   return AWS_ERR_PARAM;
    // }
    // if (src_height <= 0) {
    //   AWS_LOGSTREAM_ERROR(__func__, "Invalid video source height " << src_height << "!");
    //   return AWS_ERR_PARAM;
    // }
    // if (dst_width <= 0) {
    //   AWS_LOGSTREAM_ERROR(__func__, "Invalid output video width " << dst_width << "!");
    //   return AWS_ERR_PARAM;
    // }
    // if (dst_height <= 0) {
    //   AWS_LOGSTREAM_ERROR(__func__, "Invalid video source height " << dst_height << "!");
    //   return AWS_ERR_PARAM;
    // }
    // if (fps_num <= 0) {
    //   AWS_LOGSTREAM_ERROR(__func__, "Invalid FPS numerator " << fps_num << "!");
    //   return AWS_ERR_PARAM;
    // }
    // if (fps_den <= 0) {
    //   AWS_LOGSTREAM_ERROR(__func__, "Invalid FPS denominator " << fps_den << "!");
    //   return AWS_ERR_PARAM;
    // }
    // if (bitrate <= 0) {
    //   AWS_LOGSTREAM_ERROR(__func__, "Invalid bit rate " << bitrate << "!");
    //   return AWS_ERR_PARAM;
    // }

    // src_width_ = src_width;
    // src_height_ = src_height;
    // src_encoding_ = src_encoding;
    // if (src_encoding_ == AV_PIX_FMT_RGB24) {
    //   src_stride_ = 3 * src_width_;  // 3 color channels (red, green, blue)
    // } else if (src_encoding_ == AV_PIX_FMT_BGR24) {
    //   src_stride_ = 3 * src_width_;  // 3 color channels (blue, green, red)
    // } else if (src_encoding_ == AV_PIX_FMT_RGBA) {
    //   src_stride_ = 4 * src_width_;  // 4 color channels (red, green, blue, alpha)
    // } else if (src_encoding_ == AV_PIX_FMT_BGRA) {
    //   src_stride_ = 4 * src_width_;  // 4 color channels (blue, green, red, alpha)
    // } else {
    //   AWS_LOG_ERROR(__func__, "Trying to work with unsupported encoding!");
    //   return AWS_ERR_PARAM;
    // }

    // dst_width_ = dst_width;
    // dst_height_ = dst_height;
    // fps_num_ = fps_num;
    // fps_den_ = fps_den;
    // bitrate_ = bitrate;

    avcodec_register_all();

    /* find the mpeg1 video decoder */
    AVCodec * codec = nullptr;
    AVDictionary * opts = nullptr;
    if (codec_name.empty()) {
      codec = avcodec_find_decoder_by_name(kDefaultHardwareCodec);
      if (AWS_ERR_OK != open_codec(codec, opts)) {
        codec = avcodec_find_decoder_by_name(kDefaultSoftwareCodec);
	      // av_dict_set(&opts, "preset", "veryfast", 0);
        // av_dict_set(&opts, "tune", "zerolatency", 0);

        if (AWS_ERR_OK != open_codec(codec, opts)) {
          AWS_LOGSTREAM_ERROR(__func__, kDefaultHardwareCodec << " and " << kDefaultSoftwareCodec
                                                              << " codecs were not available!");
          return AWS_ERR_NOT_FOUND;
        }
      }
    } else {
      codec = avcodec_find_decoder_by_name(codec_name.c_str());
      if (codec == nullptr) {
        AWS_LOGSTREAM_ERROR(__func__, "Null codec");
      }
      if (AWS_ERR_OK != open_codec(codec, opts)) {
        AWS_LOGSTREAM_ERROR(__func__, codec_name << " codec not found!");
        return AWS_ERR_NOT_FOUND;
      }
    }
    AWS_LOGSTREAM_INFO(__func__, "Encoding using " << codec->name << " codec");
    


    // dst_width_ = param_->width;
    // dst_height_ = param_->height;

    // pic_in_ = av_frame_alloc();
    // if (nullptr == pic_in_) {
    //   AWS_LOG_ERROR(__func__, "Could not allocate video frame");
    //   return AWS_ERR_MEMORY;
    // }
    // pic_in_->format = param_->pix_fmt;
    // pic_in_->width = param_->width;
    // pic_in_->height = param_->height;
    // pic_in_->pts = 0;

    /* the image can be allocated by any means and av_image_alloc() is
     * just the most convenient way if av_malloc() is to be used */
    // int ret = av_image_alloc(pic_in_->data, pic_in_->linesize, param_->width, param_->height,
    //                          param_->pix_fmt, 32);
    // if (ret < 0) {
    //   AWS_LOGSTREAM_ERROR(__func__,
    //                       "Could not allocate raw picture buffer"
    //                       " (av_image_alloc() returned: "
    //                         << ret << ")");
    //   return AWS_ERR_MEMORY;
    // }

    // convert_ctx_ = sws_getContext(src_width_, src_height_, src_encoding_, dst_width_, dst_height_,
    //                               AV_PIX_FMT_YUV420P, SWS_FAST_BILINEAR, nullptr, nullptr, nullptr);

    return AWS_ERR_OK;
  }

  ~H264DecoderImpl()
  {
    if (nullptr != convert_ctx_) {
      sws_freeContext(convert_ctx_);
    }

    if (nullptr != param_) {
      avcodec_close(param_);
      av_free(param_);
    }
  }

  AwsError Decode(const H264DecoderInput & img_data, H264DecoderOutput& res)
  {
    const AVPacket pkt = {nullptr, img_data.frame_pts, img_data.frame_dts, const_cast<uint8_t*>(img_data.frame_data.data()), img_data.frame_data.size(), 0, 0, nullptr, 0, img_data.frame_duration, -1, 0};
    AVFrame* pic_out = av_frame_alloc();

    /* Convert from image encoding to YUV420P */
    // const uint8_t * buf_in[4] = {img_data, nullptr, nullptr, nullptr};
    // sws_scale(convert_ctx_, (const uint8_t * const *)buf_in, &src_stride_, 0, src_height_,
    //           pic_in_->data, pic_in_->linesize);

    /* Encode */
    // av_init_packet(&pkt);
    // pkt.data = &img_data.frame_data[0];  // packet data will be allocated by the encoder
    // pkt.size = img_data.frame_data.size();
    // pkt.dts = img_data.frame_dts;
    // pkt.pts = img_data.frame_pts;
    // pkt.duration = img_data.frame_duration;
    // pkt.flags |= img_data.key_frame & AV_PKT_FLAG_KEY;

    int got_output = 0;

    int ret = avcodec_decode_video2(param_, pic_out, &got_output, &pkt);
    
    // ++pic_in_->pts;
    if (ret < 0) {
      AWS_LOGSTREAM_ERROR(__func__,
                          "Error encoding frame (avcodec_encode_video2() returned: " << ret << ")");
      return AWS_ERR_FAILURE;
    }
    if (got_output) {
      if (convert_ctx_ == nullptr) {
        convert_ctx_ = sws_getContext(pic_out->width, pic_out->height, static_cast<AVPixelFormat>(pic_out->format), pic_out->width, pic_out->height,
                                  AV_PIX_FMT_BGR24, 0, nullptr, nullptr, nullptr);
      }
      // const uint8_t * buf_in[1] = {pic_out->data};
      res.frame_data.resize(3 * pic_out->height * pic_out->width);
      uint8_t * out_data_ptr[1] = { res.frame_data.data() };
      int output_stride = pic_out->width * 3;
      sws_scale(convert_ctx_, pic_out->data, pic_out->linesize, 0, pic_out->height,
              out_data_ptr, &output_stride);
      res.encoding = AV_PIX_FMT_BGR24;
      res.height = pic_out->height;
      res.width = pic_out->width;
      res.stride = output_stride;
      av_frame_free(&pic_out);
      return AWS_ERR_OK;
    }

    return AWS_ERR_EMPTY;
  }

private:
  int width_;
  int height_;
  AVPixelFormat dst_encoding_;
  int dst_stride_;
  struct SwsContext * convert_ctx_;

  AVCodecContext * param_;
  // AVFrame * pic_in_;
};

H264Decoder::H264Decoder() {}

H264Decoder::~H264Decoder() {}

AwsError H264Decoder::Initialize(const AVPixelFormat dst_encoding,
                                 const ParameterReaderInterface & dst_params)
{
  std::string codec;
  dst_params.ReadParam(ParameterPath(kCodecKey), codec);

  impl_ = std::unique_ptr<H264DecoderImpl>(new H264DecoderImpl());

  return impl_->Initialize(codec);
}

AwsError H264Decoder::Decode(const H264DecoderInput & img_data, H264DecoderOutput& res) const
{
  return impl_->Decode(img_data, res);
}

}  // namespace Encoding
}  // namespace Utils
}  // namespace Aws
