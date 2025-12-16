---
title: Voice Commands with Whisper
---

# 🎤 Voice Commands with Whisper

The most natural way for a human to give a command is by speaking. To enable this, our robot needs a "digital ear"—a system that can convert spoken language into text. For this task, we will use **OpenAI's Whisper**, a state-of-the-art, open-source model for Automatic Speech Recognition (ASR).

### Why Whisper?

Whisper is an excellent choice for robotics applications because it is:
-   **Highly Accurate:** It has been trained on a massive and diverse dataset of audio, making it robust to accents, background noise, and technical language.
-   **Multilingual:** It can transcribe speech from dozens of languages.
-   **Flexible:** You can use the simple OpenAI API for a managed solution or run the open-source model locally on your own hardware for more control and offline capabilities.

### The ASR Pipeline in ROS

Integrating a voice command system into our ROS 2 robot involves a few key steps, which can be encapsulated in a single ROS node:

1.  **Capture Audio:** The node subscribes to an audio stream. In a real robot, this would come from a microphone driver. In simulation, you can stream audio from your computer's microphone.
2.  **Detect Speech:** The node runs a simple Voice Activity Detection (VAD) algorithm to detect when a user starts and stops speaking. This avoids transcribing silence or background noise.
3.  **Transcribe:** Once speech is detected, the audio segment is sent to the Whisper model (either via API or locally).
4.  **Publish Text:** The transcribed text is then published as a `std_msgs/String` message to a ROS 2 topic, such as `/voice_command`.

![ASR Pipeline](https://developer.nvidia.com/blog/wp-content/uploads/2022/10/audio-data-collection-and-preparation-pipeline.png)
*A conceptual pipeline for processing audio data.*

```python
# Conceptual Python code for a voice command node
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int16MultiArray
import openai # Using the OpenAI API for Whisper

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')
        # Publisher for the transcribed text
        self.publisher_ = self.create_publisher(String, 'voice_command', 10)
        # Subscriber to the raw audio stream
        self.subscription = self.create_subscription(
            Int16MultiArray, 'audio_stream', self.audio_callback, 10)
        self.get_logger().info("Voice Command node started.")
        
    def audio_callback(self, msg):
        # In a real implementation:
        # 1. Buffer audio data.
        # 2. Use a VAD to detect speech.
        # 3. When speech ends, send the buffer to Whisper.
        audio_data = self.convert_msg_to_audio_format(msg)

        # Conceptual: Call to Whisper API
        # transcript = openai.Audio.transcribe("whisper-1", audio_data)
        transcript_text = "Example command: pick up the block" # Placeholder

        # 4. Publish the transcript.
        command_msg = String()
        command_msg.data = transcript_text
        self.publisher_.publish(command_msg)
        self.get_logger().info(f'Published command: "{command_msg.data}"')

    def convert_msg_to_audio_format(self, msg):
        # Logic to convert ROS message to a format Whisper can use
        # (e.g., a temporary .wav file)
        pass
```
This node acts as a bridge, converting the analog world of sound into the digital world of text that our robot's planning system can understand.
