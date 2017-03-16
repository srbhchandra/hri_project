import pyttsx

def onStart(name):
	print 'Starting', name

engine = pyttsx.init()
voices = engine.getProperty('voices')
rate = engine.getProperty('rate')
engine.setProperty('rate', 120)
engine.connect('started-utterance', onStart)
#good_id = [2, ]
for id, voice in enumerate(voices):
	print id
	if id >= 10:
		engine.setProperty('voice', voice.id) #change index to change voices
		engine.say("Hey Aaron, you have a message and an item from Saurabh")
engine.runAndWait()

