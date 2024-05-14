from moviepy.editor import VideoFileClip

videoClip = VideoFileClip("autopilot_demo(1).mp4")
scaled_clip = videoClip.resize(0.2)
scaled_clip.write_gif("autopilot_demo(2).gif")
