from aiortc.contrib.media import MediaPlayer, MediaRecorder


class AudioDevices:

    def microphone(self):
        """
        USB microphone

        Change hw:1,0 if needed.
        """
        return MediaPlayer(
            "hw:1,0",
            format="alsa",
            options={
                "channels": "1",
                "sample_rate": "48000"
            }
        )

    def speaker(self):
        """
        USB speaker
        """
        return MediaRecorder(
            "hw:1,0",
            format="alsa"
        )


audio = AudioDevices()
