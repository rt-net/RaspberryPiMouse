#!/usr/bin/python
import sys
file = open('/dev/input/js0','r')
data = []
led0 = open('/dev/rtled0','a')
led1 = open('/dev/rtled1','a')
led2 = open('/dev/rtled2','a')
led3 = open('/dev/rtled3','a')
rmotor = open('/dev/rtmotor_raw_r0','a')
lmotor = open('/dev/rtmotor_raw_l0','a')
motoren = open('/dev/rtmotoren0','a')
buzzer = open('/dev/rtbuzzer0','a')
straight = 0
curbe =0

while 1:
	for character in file.read(1):
		data += ['%02X' % ord(character)]
 		if len(data) == 8:
			if data[6] == '01':
				if data[4] == '01':
	 				if data[7] == '0F':
 						led3.write("1")
 					elif data[7] == '0E':
 						led2.write("1")
					elif data[7] == '0D':
 						led1.write("1")
 					elif data[7] == '0C':
 						led0.write("1")
					elif data[7] == '04':
						print >> motoren,1
 						print >> rmotor,500
						print >> lmotor,500
 					elif data[7] == '06':
 						print >> motoren,1
						print >> rmotor,-500
						print >> lmotor,-500
 					elif data[7] == '05':
 						print >> motoren,1
 						print >> rmotor,-500
						print >> lmotor,500
					elif data[7] == '07':
						print >> motoren,1
						print >> rmotor,500
						print >> lmotor,-500
					elif data[7] == '00' :
						print >> buzzer,440
				elif data[4] == '00':
					if data[7] == '0F':
                                                led3.write("0")
                                        elif data[7] == '0E':
                                                led2.write("0")
                                        elif data[7] == '0D':
                                                led1.write("0")
                                        elif data[7] == '0C':
                                                led0.write("0")
                                        elif data[7] == '04':
                                                print >> motoren,0
                                        elif data[7] == '06':
                                                print >> motoren,0
                                        elif data[7] == '05':
                                                print >> motoren,0
                                        elif data[7] == '07':
                                                print >> motoren,0
                                        elif data[7] == '00' :
                                                print >> buzzer,0
                                straight=0
                                curbe=0	
                        elif data[6] == '02':#analog
                                if data[7] == '01':#left
                                        print >> motoren,1
                                        if int(data[5],16) > 127:
                                                if int(data[5],16) < 252:
                                                        straight = (255 - int(data[5],16)) * 10
							print "straight %d" % straight
                                                        print >> rmotor,int(straight * (127 - curbe) / 127)
                                                        print >> lmotor,int(straight * (127 + curbe) / 127)
                                        elif int(data[5],16)  > 3:
                                                straight = -(int(data[5],16)) * 10
                                                print "straight %d" % straight
                                                print >> rmotor,int(straight * (127 - curbe ) /127)
                                                print >> lmotor,int(straight * (127 + curbe) / 127)
                                        else :
						print >> motoren,0
                                		print >> lmotor,0
                                		print >> rmotor,0
                                                straight=0
                                elif data[7] == '02':#right
                                        print >> motoren,1
                                        if int(data[5],16) > 127:#left
                                                if int(data[5],16) < 252:
                                                        curbe = -(255 - int(data[5],16))
							print "curbe %d" % curbe
                                                        print >> rmotor,int(straight * (127 -curbe) /127)
                                                        print >> lmotor,int(straight * (127 + curbe) /127)
                                        elif int(data[5],16) > 3:
                                                curbe = (int(data[5],16))
                                                print "curbe %d" % curbe
                                                print >> rmotor,int(straight * (127 - curbe) / 127)
                                                print >> lmotor,int(straight * (127 + curbe) / 127)
                                        else :
                                                curbe=0			
			led3.flush()
                  	led2.flush()
                       	led1.flush()
                       	led0.flush()
                       	motoren.flush()
                       	rmotor.flush()
                        lmotor.flush()
			buzzer.flush()
			data = []
