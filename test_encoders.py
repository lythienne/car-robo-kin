import time #used to set delay time to control moving distance

#set up Raspberry Pi GPIO
import RPi.GPIO as GPIO #control through GPIO pins

GPIO.setmode(GPIO.BCM)

#Encoders, GPIO.board pin
S1FR = 17 #pin 11
S2FR = 27 #pin 13 

S1FL = 22 #pin 15
S2FL = 10 #pin 19

S1RR = 9  #pin 21
S2RR = 11 #pin 23

S1RL = 5  #pin 29
S2RL = 6  #pin 31

#encoder class 
class Encoder:
  def __init__(self, name, S1, S2):
    self.name = name #for debug
    self.s1  = S1 #pin
    GPIO.setup(S1, GPIO.IN) #instance
    self.s2  = S2 #pin
    GPIO.setup(S2, GPIO.IN)  
    self.aState = 0 #value (aState)
    self.bState = 0
    self.aLastState = 0 # remember last value (aLastState)
    self.bLastState = 0 # not needed?
    self.counter = 0
  	
  def read(self):
    self.aState  = GPIO.input(self.s1)
    self.bState  = GPIO.input(self.s2)
    return self.aState, self.bState
    
  def name(self):
    return self.name
    
#end of Encoder class 

#Set up Encoder instances with connections, GPIO.board (swheel)
sfl = Encoder("sfl", S1FL, S2FL)
sfr = Encoder("sfr", S1FR, S2FR)
srl = Encoder("srl", S1RL, S2RL)
srr = Encoder("srr", S1RR, S2RR)

def readEncoder(swheel):
  #Reads the "current" state of the encoders
  aState, bState = swheel.read() 
  #counter = swheel.counter # should not be needed, but += -=?
  #If the previous and the current state are different,  a Pulse has occured
  if aState != swheel.aLastState  :
     #If the outputB state is different to the outputA state, rotating clockwise
     if bState!= aState :
       swheel.counter += 1
     else :
       swheel.counter -= 1
     print(swheel.name +" position: " + str(swheel.counter))
     
  swheel.aLastState = aState #remember last state of a
  #swheel.counter = counter #remember position
  
def test_Encoders():
  readEncoder(sfl) 
  readEncoder(sfr) 
  readEncoder(srl) 
  readEncoder(srr) 


def destroy():
    GPIO.cleanup()
    
def main(): 
    print("starting main")
    while True:
      test_Encoders()
  
if __name__ == '__main__':
    try:
      main()
    except KeyboardInterrupt:
      destroy()  #clean up GPIO
      print("Done")    