import pyttsx
import os

def onStart(name):
	print 'Starting', name

engine = pyttsx.init()
voices = engine.getProperty('voices')
rate = engine.getProperty('rate')
engine.setProperty('rate', 120)
engine.connect('started-utterance', onStart)
engine.startLoop(False)
#good_id = [2, ]
for id, voice in enumerate(voices):
	break
	if id > 10:
		engine.setProperty('voice', voice.id) #change index to change voices
		engine.say("Hey Aaron, you have\n a message and an\n item from Saurabh")
#		engine.iterate()
		engine.runAndWait()
#engine.endLoop()

msg = 'Hey Aaron, you have\n a message and an\n item from Saurabh'
msg1 = "espeak '" + msg + "'"
os.system("espeak 'The quick brown fox'")
print msg1
os.system(msg1)
os.system(msg1)
os.system(msg1)
os.system(msg1)
os.system(msg1)
os.system(msg1)
os.system(msg1)
