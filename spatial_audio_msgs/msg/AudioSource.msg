uint8 AUDIO_SOURCE_STATE_STOP=0
uint8 AUDIO_SOURCE_STATE_PLAYING=1
uint8 AUDIO_SOURCE_STATE_PAUSE=2

Header header
int32 audio_source_id
geometry_msgs/Pose pose
string stream_topic_audio
string stream_topic_info
int32 state
