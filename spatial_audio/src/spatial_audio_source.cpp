// Standaerd C++ Library
#include <iostream>
// Boost
#include <boost/shared_ptr.hpp>
// OpenAL headers
#include <AL/al.h>
#include <AL/alc.h>
// ROS
#include <audio_stream_msgs/AudioData.h>
#include <audio_stream_msgs/AudioInfo.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <spatial_audio_msgs/PlaySpatialAudio.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
// User
#include <spatial_audio/spatial_audio_source.h>
#include <spatial_audio/util.h>

namespace spatial_audio
{
SpatialAudioSource::SpatialAudioSource()
{
  this->is_init_ = false;
  this->is_playing_ = false;
}

SpatialAudioSource::SpatialAudioSource(ros::NodeHandle& nh, int id, std::string reference_frame_id,
                                       geometry_msgs::Pose source_pose, std::string stream_topic_info,
                                       std::string stream_topic_audio, bool auto_play, int initial_buffer_num,
                                       double timeout)
{
  this->is_init_ = false;
  this->is_playing_ = false;
  if (not this->init(nh, id, reference_frame_id, source_pose, stream_topic_info, stream_topic_audio, auto_play,
                     initial_buffer_num, timeout))
  {
    this->is_init_ = false;
  }
  else
  {
    this->is_init_ = true;
  }
}

SpatialAudioSource::~SpatialAudioSource()
{
  // stop playing
  if (this->is_init_)
  {
    this->close();
  }
}

bool SpatialAudioSource::init(ros::NodeHandle& nh, int id, std::string reference_frame_id,
                              geometry_msgs::Pose source_pose, std::string stream_topic_info,
                              std::string stream_topic_audio, bool auto_play, int initial_buffer_num, double timeout)
{
  bool ret = true;

  // すでに initializa 済みであればエラーを返す
  if (this->is_init_)
  {
    return false;
  }

  // initialization
  /**
   * initialization of members
   */
  this->id_ = id;
  this->source_frame_id_ = reference_frame_id;
  this->source_pose_ = source_pose;
  /**
   * Get a audio info message for meta information of audio stream
   */
  audio_stream_msgs::AudioInfo::ConstPtr info =
      ros::topic::waitForMessage<audio_stream_msgs::AudioInfo>(stream_topic_info, ros::Duration(timeout));
  if (not info)
  {
    ROS_ERROR("Cannot retrive a message from info topic: %s", stream_topic_info.c_str());
    return false;
  }
  if (info->channels == audio_stream_msgs::AudioInfo::AUDIOINFO_CHANNELS_MONAURAL &&
      info->width == audio_stream_msgs::AudioInfo::AUDIOINFO_WIDTH_8BIT)
  {
    this->stream_format_ = AL_FORMAT_MONO8;
  }
  else if (info->channels == audio_stream_msgs::AudioInfo::AUDIOINFO_CHANNELS_MONAURAL &&
           info->width == audio_stream_msgs::AudioInfo::AUDIOINFO_WIDTH_16BIT)
  {
    this->stream_format_ = AL_FORMAT_MONO16;
  }
  else if (info->channels == audio_stream_msgs::AudioInfo::AUDIOINFO_CHANNELS_STEREO &&
           info->width == audio_stream_msgs::AudioInfo::AUDIOINFO_WIDTH_8BIT)
  {
    this->stream_format_ = AL_FORMAT_STEREO8;
  }
  else if (info->channels == audio_stream_msgs::AudioInfo::AUDIOINFO_CHANNELS_STEREO &&
           info->width == audio_stream_msgs::AudioInfo::AUDIOINFO_WIDTH_16BIT)
  {
    this->stream_format_ = AL_FORMAT_STEREO16;
  }
  else
  {
    ROS_ERROR("Invalid channels or bit per sample in rosservice.");
    return false;
  }
  this->stream_sampling_rate_ = info->sampling_rate;
  /**
   * generating OpenAL resources
   */
  // generate a source object
  alGenSources((ALuint)1, &this->al_source_id_);
  alSourcef(this->al_source_id_, AL_PITCH, 1);
  alSourcef(this->al_source_id_, AL_GAIN, 1);
  // AL_LOOPING should be AL_FALSE with stream source.
  // referece: https://stackoverflow.com/questions/6990701/why-would-alsourceunqueuebuffers-fail-with-invalid-operation
  alSourcei(this->al_source_id_, AL_LOOPING, AL_FALSE);
  /**
   * Subscribe the audio stream topic
   */
  this->stream_subscriber_ =
      nh.subscribe<audio_stream_msgs::AudioData>(stream_topic_audio, 1, &SpatialAudioSource::callbackAudioStream, this);
  /**
   * start playing if auto_play arg is true
   */
  if (auto_play)
  {
    /**
     * Start Buffering
     */
    this->is_playing_ = true;
    /**
     * Wait until buffering
     */
    ALsizei n = 0;
    while (n < initial_buffer_num)
    {
      ros::Duration(1.0).sleep();
      alGetSourcei(this->al_source_id_, AL_BUFFERS_QUEUED, &n);
      ROS_INFO("waiting for buffering until %d buffers for id: %d with topic: %s, current buffers: %d...",
               initial_buffer_num, this->id_, stream_topic_audio.c_str(), n);
    }
    // start actual playing
    alSourcePlay(this->al_source_id_);
  }
  /**
   * Debug print
   */
  ROS_INFO("Add an audio source object. id: %d, frame_id: %s, stream topic: %s", this->id_,
           this->source_frame_id_.c_str(), stream_topic_audio.c_str());
  /**
   * return
   */
  if (ret == true)
  {
    this->is_init_ = true;
  }
  return ret;
}

void SpatialAudioSource::close()
{
  this->mtx_.lock();
  this->stream_subscriber_.shutdown();
  if (this->getSourceState() != AL_PLAYING)
  {
    alSourceStop(this->al_source_id_);
  }
  this->mtx_.unlock();
  // release buffer object
  this->dequeALBuffers();
  // release source obect
  alDeleteSources(1, &this->al_source_id_);
  this->is_init_ = false;
}

void SpatialAudioSource::verbose()
{
  ROS_INFO_STREAM("audio source id:" << this->id_ << " is_init:" << this->is_init_
                                     << " is_playing:" << this->is_playing_);
}

void SpatialAudioSource::update(std::string& reference_frame_id, geometry_msgs::Pose& source_pose,
                                std::string& stream_topic_info, std::string& stream_topic_audio)
{
  this->mtx_.lock();
  this->source_frame_id_ = reference_frame_id;
  this->source_pose_ = source_pose;
  // TODO: update process for ros topic
  this->mtx_.unlock();
}

void SpatialAudioSource::updateCoordinate(std::string& head_frame_id, tf2_ros::Buffer& tf_buffer, ALCcontext* context)
{
  geometry_msgs::TransformStamped transform_reference2head;
  geometry_msgs::Pose pose_source;
  this->mtx_.lock();
  std::string source_frame_id = this->source_frame_id_;
  geometry_msgs::Pose source_pose = this->source_pose_;
  this->mtx_.unlock();
  try
  {
    transform_reference2head = tf_buffer.lookupTransform(head_frame_id.c_str(), source_frame_id.c_str(), ros::Time(0));
  }
  catch (const tf2::LookupException& e)
  {
    ROS_ERROR("%s", e.what());
    return;
  }
  catch (const tf2::ExtrapolationException& e)
  {
    ROS_ERROR("%s", e.what());
    return;
  }
  tf2::doTransform(source_pose, pose_source, transform_reference2head);

  alcSuspendContext(context);
  {
    ALfloat pos_source[3];
    pos_source[0] = pose_source.position.x;
    pos_source[1] = pose_source.position.y;
    pos_source[2] = pose_source.position.z;
    this->mtx_.lock();
    alSourcefv(this->al_source_id_, AL_POSITION, pos_source);
    this->mtx_.unlock();
  }
  alcProcessContext(context);
}

void SpatialAudioSource::dequeALBuffers()
{
  ALsizei n, m;
  this->mtx_.lock();
  alGetSourcei(this->al_source_id_, AL_BUFFERS_PROCESSED, &n);  // get a number of buffers processed.
  ALuint* buffers = new ALuint[n];  // These buffers should be released. Maybe shared_ptr should be used.
  alSourceUnqueueBuffers(this->al_source_id_, n, buffers);  // unqueue the buffers
  alDeleteBuffers(n, buffers);
  delete[] buffers;
  alGetSourcei(this->al_source_id_, AL_BUFFERS_QUEUED, &m);
  this->mtx_.unlock();

  ROS_INFO("id:%d, %d buffers processed, %d buffers remains.", this->id_, n, m);
}

ALint SpatialAudioSource::getSourceState()
{
  ALint source_state;
  alGetSourcei(this->al_source_id_, AL_SOURCE_STATE, &source_state);
  return source_state;
}

bool SpatialAudioSource::isPlaying()
{
  return is_playing_;
}

void SpatialAudioSource::startSourcePlay()
{
  this->mtx_.lock();
  this->is_playing_ = true;
  alSourcePlay(this->al_source_id_);
  this->mtx_.unlock();
}

void SpatialAudioSource::stopSourcePlay()
{
  this->mtx_.lock();
  alSourceStop(this->al_source_id_);
  this->is_playing_ = false;
  this->mtx_.unlock();
}

int SpatialAudioSource::getAudioSourceID()
{
  return this->id_;
}

void SpatialAudioSource::callbackAudioStream(const boost::shared_ptr<audio_stream_msgs::AudioData const>& ptr_msg)
{
  this->dequeALBuffers();

  // enqueue a new buffer with recieved data
  if (this->is_playing_)
  {
    ALuint buffer_id;
    ALsizei buffer_size = ptr_msg->data.size();
    genBufferFromPCM(buffer_id, (ALvoid*)ptr_msg->data.data(), buffer_size, this->stream_sampling_rate_,
                     this->stream_format_);
    this->mtx_.lock();
    alSourceQueueBuffers(this->al_source_id_, 1, &buffer_id);
    this->mtx_.unlock();

    ROS_INFO("New data containes %d bytes.", buffer_size);
  }

  if (this->is_playing_ and this->is_init_ and this->getSourceState() != AL_PLAYING)
  {
    alSourcePlay(this->al_source_id_);
  }
}

}  // namespace spatial_audio
