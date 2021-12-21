from google.cloud import dialogflow
from time import time
import os
import pyaudio
import keyboard
import json

# keybinds
PTT_KEY = "p"
EXIT_KEY = "q"
CONFIDENCE_REQUIRED = 30

#OUT_FILE = "\\wsl$\Ubuntu-20.04\home\azm\demos\o.json"
OUT_FILE = "C:\catkin_ws\src\demons\extra\o.json"

# dialogflow settings
os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = r"C:\catkin_ws\src\demons\dialogflow\demosstt-lock-bc3e0a8aaf6f.json"
SESSION_ID = "123456789"
LOCATION_ID = "europe-west2"
CHUNK = 4096 # chunk size to send to dialogflow
# rate at which to poll audio
RATE = 16000 # DO NOT CHANGE
ENCODING = dialogflow.AudioEncoding.AUDIO_ENCODING_LINEAR_16 # dont change


def detect_intent_stream(project_id, language_code):
    # adapted from the dialogflow docs
    session_client = dialogflow.SessionsClient(client_options={"api_endpoint": f"{LOCATION_ID}-dialogflow.googleapis.com"})
    session = (f"projects/{project_id}/locations/{LOCATION_ID}/agent/sessions/{SESSION_ID}")

    #print("Session path: {}\n".format(session))
    
    p = pyaudio.PyAudio()
    stream = p.open(
        format=pyaudio.paInt16,
        channels=1,
        rate=RATE,
        input=True,
        frames_per_buffer=CHUNK)
    
    print("Listening")
    def request_generator(audio_config):
        query_input = dialogflow.QueryInput(audio_config=audio_config)

        yield dialogflow.StreamingDetectIntentRequest(
            session=session, query_input=query_input
        )
        
        while keyboard.is_pressed("p"):
            data = stream.read(CHUNK)
            yield dialogflow.StreamingDetectIntentRequest(input_audio=data)

    audio_config = dialogflow.InputAudioConfig(
        audio_encoding=ENCODING,
        language_code=language_code,
        sample_rate_hertz=RATE,
    )

    requests = request_generator(audio_config)
    responses = session_client.streaming_detect_intent(requests=requests)

    for response in responses:
        print(
            'Intermediate transcript: "{}".'.format(
                response.recognition_result.transcript
            )
        )

    query_result = response.query_result

    print("=" * 20)
    print("Query text: {}".format(query_result.query_text))
    print(
        "Detected intent: {} (confidence: {})".format(
            query_result.intent.display_name, query_result.intent_detection_confidence
        )
    )
    print("Fulfillment text: {}\n".format(query_result.fulfillment_text))

    _t = {
        "fulfillment_text": query_result.fulfillment_text,
        "intent": query_result.intent.display_name,
        "intent_detection_confidence": query_result.intent_detection_confidence,
        "query_text": query_result.query_text
    }
    if query_result.intent_detection_confidence < (CONFIDENCE_REQUIRED/100):
        print(f"Intent confidence sub {CONFIDENCE_REQUIRED}%, ignoring")
        return
    with open(OUT_FILE, 'w') as out:
        out.write(json.dumps(_t, indent=4))

if __name__ == "__main__":
    print(f"Press '{PTT_KEY}' to start listening and '{EXIT_KEY}' to exit.")
    keyboard.add_hotkey(PTT_KEY, detect_intent_stream, args=["demosstt-lock", "en-US"])
    keyboard.wait(EXIT_KEY)
