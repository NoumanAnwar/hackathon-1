---
id: chapter1
title: Voice-to-Action with OpenAI Whisper
sidebar_position: 1
---

# Chapter 1: Voice-to-Action - OpenAI Whisper for Voice Commands

Welcome to Module 4: Vision-Language-Action (VLA)! This chapter introduces the first component of our VLA system: voice-to-action using OpenAI Whisper for voice command recognition. We'll bridge natural language understanding with robotic action, enabling robots to comprehend and respond to spoken commands.

## Overview of Voice-to-Action

Voice-to-action systems convert spoken commands into executable robotic actions. This involves:
1. **Speech Recognition**: Converting audio to text (using OpenAI Whisper)
2. **Natural Language Processing**: Interpreting the meaning of commands
3. **Action Mapping**: Translating commands to specific robot behaviors
4. **Execution**: Sending commands through ROS 2 to control the robot

## OpenAI Whisper Integration

OpenAI Whisper is a state-of-the-art speech recognition model that can transcribe audio to text with high accuracy. In robotics applications, Whisper enables natural human-robot interaction through voice commands.

### Installation and Setup

```bash
pip install openai-whisper
# Alternatively, install from source for latest features
pip install git+https://github.com/openai/whisper.git
```

### Basic Whisper Usage

```python
import whisper
import rospy
from std_msgs.msg import String

# Load model (choose: 'tiny', 'base', 'small', 'medium', 'large')
model = whisper.load_model("base")

def transcribe_audio(audio_file_path):
    # Load and process audio
    result = model.transcribe(audio_file_path)
    return result["text"]

def voice_command_callback(audio_data):
    # Process incoming audio and extract text command
    command_text = transcribe_audio(audio_data)
    rospy.loginfo(f"Recognized command: {command_text}")

    # Process the command text and map to robot actions
    process_command(command_text)

def process_command(command_text):
    # Parse command and execute appropriate robot action
    if "move forward" in command_text.lower():
        # Send move forward command via ROS 2
        send_navigation_command("forward")
    elif "turn left" in command_text.lower():
        send_navigation_command("turn_left")
    # Additional commands...
```

## Voice Command Processing Pipeline

### 1. Audio Capture and Preprocessing

For real-time voice-to-action systems, we need to continuously listen for commands:

```python
import pyaudio
import wave
import threading
import queue

class VoiceCommandListener:
    def __init__(self):
        self.model = whisper.load_model("base")
        self.audio_queue = queue.Queue()
        self.setup_audio_stream()

    def setup_audio_stream(self):
        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=16000,
            input=True,
            frames_per_buffer=1024
        )

    def start_listening(self):
        threading.Thread(target=self.record_audio, daemon=True).start()
        threading.Thread(target=self.process_audio, daemon=True).start()

    def record_audio(self):
        frames = []
        while True:
            # Listen for voice activity (simplified)
            data = self.stream.read(1024)
            frames.append(data)

            # When enough audio is collected, put it in queue
            if len(frames) > 100:  # Adjust based on desired audio length
                self.audio_queue.put(frames.copy())
                frames = []

    def process_audio(self):
        while True:
            if not self.audio_queue.empty():
                frames = self.audio_queue.get()
                # Save frames as WAV for Whisper processing
                audio_file = self.save_audio(frames)
                command = self.transcribe_audio(audio_file)
                if command:
                    self.handle_command(command)

    def transcribe_audio(self, audio_file):
        result = self.model.transcribe(audio_file)
        return result["text"]

    def handle_command(self, command_text):
        rospy.loginfo(f"Processing command: {command_text}")
        self.execute_robot_action(command_text)
```

### 2. Command Interpretation

After converting speech to text, we need to interpret and classify commands:

```python
class CommandInterpreter:
    def __init__(self):
        # Define command patterns and their corresponding robot actions
        self.command_patterns = {
            'navigation': [
                (r'move forward|go forward|forward', 'move_forward'),
                (r'move backward|go backward|backward', 'move_backward'),
                (r'turn left|left|pivot left', 'turn_left'),
                (r'turn right|right|pivot right', 'turn_right'),
                (r'stop|halt|stop moving', 'stop')
            ],
            'manipulation': [
                (r'pick up|grasp|grab (.+)', 'pick_up'),
                (r'place|put (.+)', 'place_object'),
                (r'open gripper|release', 'open_gripper'),
                (r'close gripper|grab', 'close_gripper')
            ],
            'interaction': [
                (r'hello|hi|greet', 'greet'),
                (r'help|assist me', 'request_help')
            ]
        }

    def interpret_command(self, command_text):
        command_text = command_text.lower()

        for category, patterns in self.command_patterns.items():
            for pattern, action in patterns:
                import re
                match = re.search(pattern, command_text)
                if match:
                    return {
                        'action': action,
                        'category': category,
                        'arguments': match.groups() if match.groups() else [],
                        'original_text': command_text
                    }

        return {
            'action': 'unrecognized',
            'category': 'unknown',
            'arguments': [],
            'original_text': command_text
        }
```

### 3. Integration with ROS 2

Connect voice commands to ROS 2 actions:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import whisper

class VoiceToActionNode(Node):
    def __init__(self):
        super().__init__('voice_to_action_node')

        # Publisher for robot movement
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Publisher for status updates
        self.status_pub = self.create_publisher(String, '/voice_status', 10)

        # Initialize Whisper model
        self.model = whisper.load_model("base")

        # Initialize command interpreter
        self.interpreter = CommandInterpreter()

        # Setup audio capture (if running on robot)
        self.voice_listener = VoiceCommandListener()
        self.voice_listener.start_listening()

    def execute_robot_action(self, interpreted_command):
        action = interpreted_command['action']

        if action == 'move_forward':
            self.move_robot(0.5, 0.0)  # Move forward at 0.5 m/s
        elif action == 'move_backward':
            self.move_robot(-0.5, 0.0)  # Move backward at 0.5 m/s
        elif action == 'turn_left':
            self.move_robot(0.0, 0.5)  # Turn left
        elif action == 'turn_right':
            self.move_robot(0.0, -0.5)  # Turn right
        elif action == 'stop':
            self.stop_robot()
        else:
            self.get_logger().info(f"Unrecognized action: {action}")

    def move_robot(self, linear_x, angular_z):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)

    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
```

## Real-World Considerations

### 1. Environmental Audio Processing
- Background noise reduction
- Echo cancellation for indoor environments
- Audio preprocessing for better recognition accuracy

### 2. Multilingual Support
- Whisper supports multiple languages
- Language detection for mixed-language environments
- Localized command sets for different languages

### 3. Context Awareness
- Understanding of robot's current state
- Memory of previous commands and robot position
- Ambiguous command disambiguation

## Performance Optimization

### 1. Model Selection
- Choose Whisper model size based on computational resources
- Tiny/base models for edge devices
- Large models for high accuracy on powerful systems

### 2. Latency Reduction
- Use streaming audio processing
- Implement confidence thresholds
- Use faster inference techniques (e.g., optimized models)

## Troubleshooting Common Issues

- **Audio Input Problems**: Verify microphone permissions and quality
- **Recognition Accuracy**: Check audio quality and ambient noise
- **Timing Issues**: Ensure proper synchronization between audio and action execution
- **Resource Limitations**: Monitor CPU and memory usage with Whisper models

## Integration with Previous Modules

The voice-to-action system integrates with:
- **Module 1 (ROS 2)**: Commands sent via ROS 2 topics/services
- **Module 2 (Gazebo)**: Test voice commands in simulation first
- **Module 3 (Isaac)**: Leverage perception for contextual understanding

---

**Next**: In Chapter 2, we will explore cognitive planning using LLMs to translate natural language to robotic actions.
