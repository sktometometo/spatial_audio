#ifndef SPATIAL_AUDIO_SPATIAL_AUDIO_SOURCE_H__
#define SPATIAL_AUDIO_SPATIAL_AUDIO_SOURCE_H__

// Standaerd C++ Library
#include <iostream>
#include <mutex>
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
#include <tf2_ros/transform_listener.h>

namespace spatial_audio {

    class SpatialAudioSource
    {
        public:

            SpatialAudioSource();
            SpatialAudioSource( ros::NodeHandle& nh,
                                int id,
                                std::string reference_frame_id,
                                geometry_msgs::Pose source_pose,
                                std::string stream_topic_info,
                                std::string stream_topic_audio,
                                bool auto_play = true,
                                int initial_buffer_num = 1,
                                double timeout = 1.0 );
            ~SpatialAudioSource();
            /**
             * non copyable settings due to mutex
             */
            SpatialAudioSource(const SpatialAudioSource&) = delete;
            SpatialAudioSource& operator=(const SpatialAudioSource&) = delete;
            /**
             * non copyable settings due to mutex
             */
            SpatialAudioSource(SpatialAudioSource&&) = delete;
            SpatialAudioSource& operator=(SpatialAudioSource&&) = delete;
            /**
             * Initialzation function. This function must be called before an object is actually used.
             * @param[in] nh ros node handler
             * @param[in] req a request of an audio source to add
             * @param[in] auto_play flag whether to start to play the source automtically
             */
            bool init( ros::NodeHandle& nh,
                       int id,
                       std::string reference_frame_id,
                       geometry_msgs::Pose source_pose,
                       std::string stream_topic_info,
                       std::string stream_topic_audio,
                       bool auto_play = true,
                       int initial_buffer_num = 1,
                       double timeout = 1.0 );
            /**
             * Finalization function. This function should be called when an audio source is discarded.
             */
            void close();
            /**
             * print audio source information to ros logging system
             */
            void verbose();
            /**
             * update parameters of and audio source
             */
            void update(
                    std::string& reference_frame_id,
                    geometry_msgs::Pose& source_pose,
                    std::string& stream_topic_info,
                    std::string& stream_topic_audio
                    );
            /**
             * update the pose of an audio source
             * @param[in] head_frame_id frame_id of a head
             * @param[in] tf_buffer tf2_ros buffer
             * @param[in] context OpenAL context handler
             */
            void updateCoordinate( 
                        std::string& head_frame_id,
                        tf2_ros::Buffer& tf_buffer,
                        ALCcontext* context );
            /**
             * deque OpenAL buffers done
             */
            void dequeALBuffers();
            /**
             * Return OpenAL state of the source
             */
            ALint getSourceState();
            /**
             * Return is the source is playing
             */
            bool isPlaying();
            /**
             * Start playing the source
             */
            void startSourcePlay();
            /**
             * Stop playing the source
             */
            void stopSourcePlay();
            /**
             * Get Audio Source ID
             */
            int getAudioSourceID();

        private:

            /**
             * ROS callback function for buffering stream audio
             */
            void callbackAudioStream( const boost::shared_ptr<audio_stream_msgs::AudioData const>& ptr_msg );

            int id_; // audio source id
            std::mutex mtx_; // mutex for controling access to the resource
            bool is_init_; // flag if Initialzation is done
            bool is_playing_; // flag if the source is playing
            /*
             * ROS related variables
             */
            std::string source_frame_id_; // the frame_id to which this source is fixed.
            geometry_msgs::Pose source_pose_; // the pose from the origin of source_frame_id to source
            /*
             * OpenAL related variables
             */
            ALuint al_source_id_; // source id for OpenAL source object
            /**
             * Audio stream related variables
             */
            ros::Subscriber stream_subscriber_;
            ALsizei         stream_sampling_rate_;
            ALenum          stream_format_;
    };
}

#endif
